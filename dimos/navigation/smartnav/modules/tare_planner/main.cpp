// TARE Planner - dimos NativeModule port
//
// Technology-Aware Robot Exploration planner: receives registered point clouds
// and odometry, maintains a rolling occupancy grid, detects exploration
// frontiers, plans exploration paths that maximise information gain via sensor
// coverage planning, and outputs waypoints for the local planner.
//
// Original: src/exploration_planner/tare_planner/
// Authors: Chao Cao et al. (CMU), port by dimos team
//
// Key algorithm:
//   1. Receives registered point clouds and odometry
//   2. Maintains a rolling occupancy grid
//   3. Detects exploration frontiers (boundaries between explored/unexplored)
//   4. Plans exploration paths that maximise information gain
//   5. Uses sensor coverage planning to optimise exploration
//   6. Outputs waypoints for the local planner to follow

#include <atomic>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

#include "sensor_msgs/PointCloud2.hpp"
#include "nav_msgs/Odometry.hpp"
#include "geometry_msgs/PointStamped.hpp"

#ifdef USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#endif

// ============================================================================
// Signal handling
// ============================================================================
static std::atomic<bool> g_shutdown{false};
static void signal_handler(int) { g_shutdown.store(true); }

// ============================================================================
// Wall-clock helper
// ============================================================================
static double now_seconds() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(
        steady_clock::now().time_since_epoch()).count();
}

// ============================================================================
// Eigen/math helpers (minimal, no ROS geometry_msgs dependency)
// ============================================================================
using Vec3d = Eigen::Vector3d;
using Vec3i = Eigen::Vector3i;

struct Point3 {
    double x = 0, y = 0, z = 0;
};

static double point_dist(const Point3& a, const Point3& b) {
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

static double point_xy_dist(const Point3& a, const Point3& b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

// ============================================================================
// Timer utility (replaces misc_utils_ns::Timer)
// ============================================================================
class Timer {
public:
    explicit Timer(const std::string& name = "") : name_(name), duration_ms_(0) {}
    void Start() { start_ = std::chrono::steady_clock::now(); }
    void Stop(bool print = false) {
        auto end = std::chrono::steady_clock::now();
        duration_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count();
        if (print) fprintf(stderr, "[Timer] %s: %d ms\n", name_.c_str(), duration_ms_);
    }
    int GetDurationMs() const { return duration_ms_; }
private:
    std::string name_;
    std::chrono::steady_clock::time_point start_;
    int duration_ms_;
};

// ============================================================================
// Voxel grid downsampler (non-PCL fallback)
// ============================================================================
struct PointXYZI {
    float x, y, z, intensity;
};

struct VoxelKey {
    int ix, iy, iz;
    bool operator==(const VoxelKey& o) const { return ix==o.ix && iy==o.iy && iz==o.iz; }
};
struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        size_t h = std::hash<int>()(k.ix);
        h ^= std::hash<int>()(k.iy) + 0x9e3779b9 + (h<<6) + (h>>2);
        h ^= std::hash<int>()(k.iz) + 0x9e3779b9 + (h<<6) + (h>>2);
        return h;
    }
};

static void downsample_cloud(std::vector<PointXYZI>& cloud, float lx, float ly, float lz) {
    if (cloud.empty() || lx <= 0 || ly <= 0 || lz <= 0) return;
    std::unordered_map<VoxelKey, std::pair<PointXYZI, int>, VoxelKeyHash> voxels;
    for (const auto& p : cloud) {
        VoxelKey k{(int)std::floor(p.x / lx), (int)std::floor(p.y / ly), (int)std::floor(p.z / lz)};
        auto it = voxels.find(k);
        if (it == voxels.end()) {
            voxels[k] = {p, 1};
        } else {
            auto& [acc, cnt] = it->second;
            acc.x += p.x; acc.y += p.y; acc.z += p.z; acc.intensity += p.intensity;
            cnt++;
        }
    }
    cloud.clear();
    cloud.reserve(voxels.size());
    for (auto& [k, v] : voxels) {
        auto& [acc, cnt] = v;
        cloud.push_back({acc.x/cnt, acc.y/cnt, acc.z/cnt, acc.intensity/cnt});
    }
}

// ============================================================================
// 3-D Grid template (replaces grid_ns::Grid)
// ============================================================================
template<typename T>
class Grid3D {
public:
    Grid3D() : sx_(0), sy_(0), sz_(0) {}
    Grid3D(int sx, int sy, int sz, const T& init_val, Vec3d origin, Vec3d resolution)
        : sx_(sx), sy_(sy), sz_(sz), origin_(origin), res_(resolution),
          data_(sx*sy*sz, init_val) {}

    void Resize(int sx, int sy, int sz, const T& init_val, Vec3d origin, Vec3d resolution) {
        sx_ = sx; sy_ = sy; sz_ = sz;
        origin_ = origin; res_ = resolution;
        data_.assign(sx*sy*sz, init_val);
    }
    int CellNumber() const { return sx_*sy_*sz_; }
    bool InRange(const Vec3i& sub) const {
        return sub.x()>=0 && sub.x()<sx_ && sub.y()>=0 && sub.y()<sy_ && sub.z()>=0 && sub.z()<sz_;
    }
    bool InRange(int ind) const { return ind >= 0 && ind < (int)data_.size(); }
    int Sub2Ind(const Vec3i& s) const { return s.x() + s.y()*sx_ + s.z()*sx_*sy_; }
    int Sub2Ind(int x, int y, int z) const { return x + y*sx_ + z*sx_*sy_; }
    Vec3i Ind2Sub(int ind) const {
        int z = ind / (sx_*sy_);
        int rem = ind % (sx_*sy_);
        int y = rem / sx_;
        int x = rem % sx_;
        return Vec3i(x,y,z);
    }
    Vec3d Ind2Pos(int ind) const {
        Vec3i s = Ind2Sub(ind);
        return Vec3d(origin_.x() + (s.x()+0.5)*res_.x(),
                     origin_.y() + (s.y()+0.5)*res_.y(),
                     origin_.z() + (s.z()+0.5)*res_.z());
    }
    Vec3i Pos2Sub(const Vec3d& pos) const {
        return Vec3i((int)std::floor((pos.x()-origin_.x())/res_.x()),
                     (int)std::floor((pos.y()-origin_.y())/res_.y()),
                     (int)std::floor((pos.z()-origin_.z())/res_.z()));
    }
    Vec3d Sub2Pos(const Vec3i& s) const {
        return Vec3d(origin_.x() + (s.x()+0.5)*res_.x(),
                     origin_.y() + (s.y()+0.5)*res_.y(),
                     origin_.z() + (s.z()+0.5)*res_.z());
    }
    T& At(int ind) { return data_[ind]; }
    const T& At(int ind) const { return data_[ind]; }
    void Set(int ind, const T& v) { data_[ind] = v; }
    Vec3i Size() const { return Vec3i(sx_,sy_,sz_); }
    Vec3d Origin() const { return origin_; }
    Vec3d Resolution() const { return res_; }
    void SetOrigin(const Vec3d& o) { origin_ = o; }
private:
    int sx_, sy_, sz_;
    Vec3d origin_, res_;
    std::vector<T> data_;
};

// ============================================================================
// Rolling Grid (index indirection for rolling arrays)
// ============================================================================
class RollingGrid {
public:
    RollingGrid() : sx_(0), sy_(0), sz_(0) {}
    RollingGrid(const Vec3i& size)
        : sx_(size.x()), sy_(size.y()), sz_(size.z()),
          offset_(0,0,0)
    {
        int n = sx_*sy_*sz_;
        ind_map_.resize(n);
        std::iota(ind_map_.begin(), ind_map_.end(), 0);
    }

    int GetArrayInd(int grid_ind) const {
        Vec3i sub = GridInd2Sub(grid_ind);
        Vec3i arr_sub;
        for (int i = 0; i < 3; i++) {
            int s = (i==0?sx_:(i==1?sy_:sz_));
            arr_sub(i) = ((sub(i) + offset_(i)) % s + s) % s;
        }
        return arr_sub.x() + arr_sub.y()*sx_ + arr_sub.z()*sx_*sy_;
    }
    int GetArrayInd(const Vec3i& sub) const {
        return GetArrayInd(sub.x() + sub.y()*sx_ + sub.z()*sx_*sy_);
    }

    void Roll(const Vec3i& step, std::vector<int>& rolled_out, std::vector<int>& updated) {
        rolled_out.clear();
        updated.clear();
        // Collect indices that will be rolled out
        int total = sx_*sy_*sz_;
        for (int ind = 0; ind < total; ind++) {
            Vec3i sub = GridInd2Sub(ind);
            bool out = false;
            for (int d = 0; d < 3; d++) {
                int s = (d==0?sx_:(d==1?sy_:sz_));
                int new_s = sub(d) + step(d);
                if (new_s < 0 || new_s >= s) { out = true; break; }
            }
            if (out) rolled_out.push_back(ind);
        }
        // Apply the offset
        for (int d = 0; d < 3; d++) {
            int s = (d==0?sx_:(d==1?sy_:sz_));
            offset_(d) = ((offset_(d) - step(d)) % s + s) % s;
        }
        // Collect updated array indices
        for (int ind : rolled_out) {
            updated.push_back(GetArrayInd(ind));
        }
    }

private:
    Vec3i GridInd2Sub(int ind) const {
        int z = ind / (sx_*sy_);
        int rem = ind % (sx_*sy_);
        int y = rem / sx_;
        int x = rem % sx_;
        return Vec3i(x,y,z);
    }
    int sx_, sy_, sz_;
    Vec3i offset_;
    std::vector<int> ind_map_;
};

// ============================================================================
// Rolling Occupancy Grid
// ============================================================================
enum class OccState : char { UNKNOWN = 0, OCCUPIED = 1, FREE = 2 };

class RollingOccupancyGrid {
public:
    RollingOccupancyGrid() : initialized_(false) {}

    void Init(double cell_size, double cell_height, int neighbor_num,
              double res_x, double res_y, double res_z) {
        Vec3d range(cell_size * neighbor_num, cell_size * neighbor_num, cell_height * neighbor_num);
        res_ = Vec3d(res_x, res_y, res_z);
        for (int i = 0; i < 3; i++)
            grid_size_(i) = (int)(range(i) / res_(i));
        rollover_step_ = Vec3i((int)(cell_size/res_x), (int)(cell_size/res_y), (int)(cell_height/res_z));
        origin_ = -range / 2.0;
        grid_.Resize(grid_size_.x(), grid_size_.y(), grid_size_.z(), OccState::UNKNOWN, origin_, res_);
        rolling_ = RollingGrid(grid_size_);
    }

    void InitializeOrigin(const Vec3d& origin) {
        if (!initialized_) {
            initialized_ = true;
            origin_ = origin;
            grid_.SetOrigin(origin_);
        }
    }

    bool UpdateRobotPosition(const Vec3d& robot_pos) {
        if (!initialized_) return false;
        Vec3d diff = robot_pos - origin_;
        Vec3i robot_grid_sub;
        for (int i = 0; i < 3; i++) {
            double step = rollover_step_(i) * res_(i);
            robot_grid_sub(i) = diff(i) > 0 ? (int)(diff(i) / step) : -1;
        }
        Vec3i sub_diff;
        for (int i = 0; i < 3; i++)
            sub_diff(i) = (grid_size_(i) / rollover_step_(i)) / 2 - robot_grid_sub(i);
        if (sub_diff.x()==0 && sub_diff.y()==0 && sub_diff.z()==0) return false;

        Vec3i rollover_step(0,0,0);
        for (int i = 0; i < 3; i++) {
            if (std::abs(sub_diff(i)) > 0)
                rollover_step(i) = rollover_step_(i) * (sub_diff(i)>0?1:-1) * std::abs(sub_diff(i));
        }

        std::vector<int> rolled_out, updated;
        rolling_.Roll(rollover_step, rolled_out, updated);

        // Update origin
        for (int i = 0; i < 3; i++)
            origin_(i) -= rollover_step(i) * res_(i);
        grid_.SetOrigin(origin_);

        // Reset rolled-in cells
        for (int arr_ind : updated)
            if (grid_.InRange(arr_ind)) grid_.Set(arr_ind, OccState::UNKNOWN);

        return true;
    }

    void UpdateOccupancy(const std::vector<PointXYZI>& cloud) {
        if (!initialized_) return;
        updated_indices_.clear();
        for (const auto& p : cloud) {
            Vec3i sub = grid_.Pos2Sub(Vec3d(p.x, p.y, p.z));
            if (!grid_.InRange(sub)) continue;
            int ind = grid_.Sub2Ind(sub);
            int arr_ind = rolling_.GetArrayInd(ind);
            if (grid_.InRange(arr_ind)) {
                grid_.Set(arr_ind, OccState::OCCUPIED);
                updated_indices_.push_back(ind);
            }
        }
    }

    // Simple ray tracing from origin through occupied cells
    void RayTrace(const Vec3d& origin) {
        if (!initialized_) return;
        Vec3i origin_sub = grid_.Pos2Sub(origin);
        if (!grid_.InRange(origin_sub)) return;

        // Uniquify
        std::sort(updated_indices_.begin(), updated_indices_.end());
        updated_indices_.erase(std::unique(updated_indices_.begin(), updated_indices_.end()), updated_indices_.end());

        for (int ind : updated_indices_) {
            if (!grid_.InRange(ind)) continue;
            Vec3i end_sub = grid_.Ind2Sub(ind);
            int arr_ind = rolling_.GetArrayInd(ind);
            if (!grid_.InRange(arr_ind) || grid_.At(arr_ind) != OccState::OCCUPIED) continue;

            // Bresenham-like ray cast
            Vec3i diff = end_sub - origin_sub;
            int steps = std::max({std::abs(diff.x()), std::abs(diff.y()), std::abs(diff.z()), 1});
            for (int s = 1; s < steps; s++) {
                Vec3i cur(origin_sub.x() + diff.x()*s/steps,
                          origin_sub.y() + diff.y()*s/steps,
                          origin_sub.z() + diff.z()*s/steps);
                if (!grid_.InRange(cur)) break;
                int cur_arr = rolling_.GetArrayInd(cur);
                if (!grid_.InRange(cur_arr)) break;
                if (grid_.At(cur_arr) == OccState::OCCUPIED) break;
                grid_.Set(cur_arr, OccState::FREE);
            }
        }
    }

    // Extract frontier cells: UNKNOWN cells adjacent to FREE cells in XY
    void GetFrontier(std::vector<PointXYZI>& frontier, const Vec3d& origin, const Vec3d& range) {
        if (!initialized_) return;
        frontier.clear();
        Vec3i sub_min = grid_.Pos2Sub(origin - range);
        Vec3i sub_max = grid_.Pos2Sub(origin + range);

        int cell_num = grid_.CellNumber();
        for (int ind = 0; ind < cell_num; ind++) {
            Vec3i cur = grid_.Ind2Sub(ind);
            if (!grid_.InRange(cur)) continue;
            // Bounds check
            bool in_range = true;
            for (int d = 0; d < 3; d++)
                if (cur(d) < sub_min(d) || cur(d) > sub_max(d)) { in_range = false; break; }
            if (!in_range) continue;

            int arr_ind = rolling_.GetArrayInd(cur);
            if (!grid_.InRange(arr_ind) || grid_.At(arr_ind) != OccState::UNKNOWN) continue;

            // Check if z-neighbors are free (skip if so - not a frontier)
            bool z_free = false;
            for (int dz : {-1, 1}) {
                Vec3i nb = cur; nb(2) += dz;
                if (grid_.InRange(nb)) {
                    int nb_arr = rolling_.GetArrayInd(nb);
                    if (grid_.InRange(nb_arr) && grid_.At(nb_arr) == OccState::FREE) { z_free = true; break; }
                }
            }
            if (z_free) continue;

            // Check if xy-neighbors are free
            bool xy_free = false;
            for (int d = 0; d < 2; d++) {
                for (int dd : {-1, 1}) {
                    Vec3i nb = cur; nb(d) += dd;
                    if (grid_.InRange(nb)) {
                        int nb_arr = rolling_.GetArrayInd(nb);
                        if (grid_.InRange(nb_arr) && grid_.At(nb_arr) == OccState::FREE) { xy_free = true; break; }
                    }
                }
                if (xy_free) break;
            }
            if (xy_free) {
                Vec3d pos = grid_.Sub2Pos(cur);
                frontier.push_back({(float)pos.x(), (float)pos.y(), (float)pos.z(), 0.0f});
            }
        }
    }

private:
    bool initialized_;
    Vec3d res_;
    Vec3i grid_size_;
    Vec3i rollover_step_;
    Vec3d origin_;
    Grid3D<OccState> grid_;
    RollingGrid rolling_;
    std::vector<int> updated_indices_;
};

// ============================================================================
// Exploration path node types
// ============================================================================
enum class NodeType {
    ROBOT              = 0,
    LOOKAHEAD_POINT    = 2,
    LOCAL_VIEWPOINT    = 4,
    LOCAL_PATH_START   = 6,
    LOCAL_PATH_END     = 8,
    LOCAL_VIA_POINT    = 10,
    GLOBAL_VIEWPOINT   = 1,
    GLOBAL_VIA_POINT   = 3,
    HOME               = 5
};

struct PathNode {
    Vec3d position = Vec3d::Zero();
    NodeType type = NodeType::LOCAL_VIA_POINT;
    int local_viewpoint_ind = -1;
    int global_subspace_index = -1;

    bool operator==(const PathNode& o) const {
        return (position - o.position).norm() < 0.2 && type == o.type;
    }
    bool operator!=(const PathNode& o) const { return !(*this == o); }
};

struct ExplorationPath {
    std::vector<PathNode> nodes;

    double GetLength() const {
        double len = 0;
        for (size_t i = 1; i < nodes.size(); i++)
            len += (nodes[i].position - nodes[i-1].position).norm();
        return len;
    }
    int GetNodeNum() const { return (int)nodes.size(); }
    void Append(const PathNode& n) {
        if (nodes.empty() || nodes.back() != n) nodes.push_back(n);
    }
    void Append(const ExplorationPath& p) {
        for (const auto& n : p.nodes) Append(n);
    }
    void Reverse() { std::reverse(nodes.begin(), nodes.end()); }
    void Reset() { nodes.clear(); }
};

// ============================================================================
// Viewpoint - simplified sensor coverage model
// ============================================================================
struct Viewpoint {
    Point3 position;
    bool in_collision = false;
    bool in_line_of_sight = true;
    bool connected = true;
    bool visited = false;
    bool selected = false;
    bool is_candidate = false;
    bool in_exploring_cell = false;
    double height = 0;
    int cell_ind = -1;
    std::vector<int> covered_points;
    std::vector<int> covered_frontier_points;

    void Reset() {
        in_collision = false; in_line_of_sight = true;
        connected = true; visited = false; selected = false;
        is_candidate = false; in_exploring_cell = false;
        covered_points.clear(); covered_frontier_points.clear();
    }
    void ResetCoverage() {
        covered_points.clear();
        covered_frontier_points.clear();
    }
};

// ============================================================================
// Greedy TSP solver (replaces OR-Tools when not available)
// ============================================================================
#ifdef USE_ORTOOLS
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#endif

static void solve_tsp_greedy(const std::vector<std::vector<int>>& dist_matrix,
                             int depot,
                             std::vector<int>& order) {
    int n = (int)dist_matrix.size();
    if (n <= 1) { order.clear(); if (n==1) order.push_back(0); return; }
    std::vector<bool> visited(n, false);
    order.clear();
    order.push_back(depot);
    visited[depot] = true;
    for (int step = 1; step < n; step++) {
        int cur = order.back();
        int best = -1;
        int best_dist = std::numeric_limits<int>::max();
        for (int j = 0; j < n; j++) {
            if (!visited[j] && dist_matrix[cur][j] < best_dist) {
                best_dist = dist_matrix[cur][j];
                best = j;
            }
        }
        if (best < 0) break;
        visited[best] = true;
        order.push_back(best);
    }
}

#ifdef USE_ORTOOLS
static void solve_tsp_ortools(const std::vector<std::vector<int>>& dist_matrix,
                              int depot,
                              std::vector<int>& order) {
    using namespace operations_research;
    int n = (int)dist_matrix.size();
    RoutingIndexManager manager(n, 1, RoutingIndexManager::NodeIndex{depot});
    RoutingModel routing(manager);
    const int cb = routing.RegisterTransitCallback(
        [&](int64_t from, int64_t to) -> int64_t {
            return dist_matrix[manager.IndexToNode(from).value()][manager.IndexToNode(to).value()];
        });
    routing.SetArcCostEvaluatorOfAllVehicles(cb);
    RoutingSearchParameters params = DefaultRoutingSearchParameters();
    params.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    const Assignment* sol = routing.SolveWithParameters(params);
    order.clear();
    if (sol) {
        int64_t idx = routing.Start(0);
        while (!routing.IsEnd(idx)) {
            order.push_back((int)manager.IndexToNode(idx).value());
            idx = sol->Value(routing.NextVar(idx));
        }
    }
}
#endif

static void solve_tsp(const std::vector<std::vector<int>>& dist_matrix,
                      int depot, std::vector<int>& order) {
#ifdef USE_ORTOOLS
    solve_tsp_ortools(dist_matrix, depot, order);
#else
    solve_tsp_greedy(dist_matrix, depot, order);
#endif
}

// ============================================================================
// Keypose Graph - simplified graph of robot key poses
// ============================================================================
struct KeyposeNode {
    Point3 position;
    int node_ind = 0;
    int keypose_id = 0;
    bool is_keypose = true;
    bool is_connected = true;
};

class KeyposeGraph {
public:
    std::vector<KeyposeNode> nodes;
    std::vector<std::vector<int>> graph;
    std::vector<std::vector<double>> dist;

    double kAddNodeMinDist = 0.5;
    double kAddNonKeyposeNodeMinDist = 0.5;
    double kAddEdgeConnectDistThr = 0.5;
    double kAddEdgeToLastKeyposeDistThr = 0.5;
    double kAddEdgeVerticalThreshold = 0.5;
    int current_keypose_id = 0;
    Point3 current_keypose_position;

    void AddNode(const Point3& pos, int node_ind, int keypose_id, bool is_kp) {
        KeyposeNode n;
        n.position = pos; n.node_ind = node_ind;
        n.keypose_id = keypose_id; n.is_keypose = is_kp;
        nodes.push_back(n);
        graph.push_back({});
        dist.push_back({});
    }
    void AddEdge(int from, int to, double d) {
        if (from < 0 || from >= (int)graph.size() || to < 0 || to >= (int)graph.size()) return;
        graph[from].push_back(to); graph[to].push_back(from);
        dist[from].push_back(d); dist[to].push_back(d);
    }
    bool HasEdgeBetween(int a, int b) const {
        if (a < 0 || a >= (int)graph.size() || b < 0 || b >= (int)graph.size()) return false;
        return std::find(graph[a].begin(), graph[a].end(), b) != graph[a].end();
    }

    int AddKeyposeNode(const Point3& pos, int keypose_id) {
        current_keypose_position = pos;
        current_keypose_id = keypose_id;
        int new_ind = (int)nodes.size();

        if (nodes.empty()) {
            AddNode(pos, new_ind, keypose_id, true);
            return new_ind;
        }

        // Find closest keypose and last keypose
        double min_dist = 1e18; int min_ind = -1;
        double last_dist = 1e18; int last_ind = -1; int max_kp_id = 0;
        for (int i = 0; i < (int)nodes.size(); i++) {
            if (!nodes[i].is_keypose) continue;
            if (std::abs(nodes[i].position.z - pos.z) > kAddEdgeVerticalThreshold) continue;
            double d = point_dist(nodes[i].position, pos);
            if (d < min_dist) { min_dist = d; min_ind = i; }
            if (nodes[i].keypose_id > max_kp_id) {
                max_kp_id = nodes[i].keypose_id;
                last_dist = d; last_ind = i;
            }
        }

        if (min_ind >= 0 && min_dist > kAddNodeMinDist) {
            if (last_dist < kAddEdgeToLastKeyposeDistThr && last_ind >= 0) {
                AddNode(pos, new_ind, keypose_id, true);
                AddEdge(last_ind, new_ind, last_dist);
            } else {
                AddNode(pos, new_ind, keypose_id, true);
                AddEdge(min_ind, new_ind, min_dist);
            }
            // Connect to other in-range nodes
            for (int i = 0; i < (int)nodes.size()-1; i++) {
                double d = point_dist(nodes[i].position, pos);
                if (d < kAddEdgeConnectDistThr && !HasEdgeBetween(new_ind, i)) {
                    AddEdge(new_ind, i, d);
                }
            }
            return new_ind;
        } else if (min_ind >= 0) {
            return min_ind;
        } else {
            AddNode(pos, new_ind, keypose_id, true);
            return new_ind;
        }
    }

    int GetClosestNodeInd(const Point3& pos) const {
        int best = -1; double best_d = 1e18;
        for (int i = 0; i < (int)nodes.size(); i++) {
            double d = point_dist(nodes[i].position, pos);
            if (d < best_d) { best_d = d; best = i; }
        }
        return best;
    }
    Point3 GetClosestNodePosition(const Point3& pos) const {
        int ind = GetClosestNodeInd(pos);
        if (ind >= 0) return nodes[ind].position;
        return Point3{0,0,0};
    }

    // Dijkstra shortest path
    double GetShortestPath(const Point3& start, const Point3& goal,
                           std::vector<Point3>& path_points) const {
        path_points.clear();
        if (nodes.size() < 2) {
            path_points.push_back(start);
            path_points.push_back(goal);
            return point_dist(start, goal);
        }
        int from = GetClosestNodeInd(start);
        int to = GetClosestNodeInd(goal);
        if (from < 0 || to < 0) return 1e18;

        int n = (int)nodes.size();
        std::vector<double> d(n, 1e18);
        std::vector<int> prev(n, -1);
        d[from] = 0;
        using PII = std::pair<double, int>;
        std::priority_queue<PII, std::vector<PII>, std::greater<PII>> pq;
        pq.push({0, from});
        while (!pq.empty()) {
            auto [cd, u] = pq.top(); pq.pop();
            if (cd > d[u]) continue;
            for (int j = 0; j < (int)graph[u].size(); j++) {
                int v = graph[u][j];
                double nd = d[u] + dist[u][j];
                if (nd < d[v]) {
                    d[v] = nd; prev[v] = u;
                    pq.push({nd, v});
                }
            }
        }
        if (d[to] >= 1e17) {
            path_points.push_back(start);
            path_points.push_back(goal);
            return point_dist(start, goal);
        }
        std::vector<int> idx;
        for (int cur = to; cur != -1; cur = prev[cur]) idx.push_back(cur);
        std::reverse(idx.begin(), idx.end());
        for (int i : idx) path_points.push_back(nodes[i].position);
        return d[to];
    }

    // Connectivity check (BFS from first keypose)
    void CheckConnectivity() {
        for (auto& n : nodes) n.is_connected = false;
        int start = -1;
        for (int i = 0; i < (int)nodes.size(); i++) {
            if (nodes[i].is_keypose) { start = i; break; }
        }
        if (start < 0) return;
        std::queue<int> q; q.push(start);
        nodes[start].is_connected = true;
        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (int v : graph[u]) {
                if (!nodes[v].is_connected) {
                    nodes[v].is_connected = true;
                    q.push(v);
                }
            }
        }
    }
};

// ============================================================================
// Grid World - maintains global exploration subspaces
// ============================================================================
enum class CellStatus { UNSEEN = 0, EXPLORING = 1, COVERED = 2, NOGO = 3 };

struct GridCell {
    Point3 center;
    CellStatus status = CellStatus::UNSEEN;
    std::vector<int> viewpoint_indices;
    int visit_count = 0;
    int keypose_id = 0;
    Vec3d viewpoint_position = Vec3d::Zero();
};

class GridWorld {
public:
    GridWorld() : initialized_(false), neighbors_init_(false), return_home_(false), set_home_(false),
                  kCellSize(6.0), kCellHeight(6.0), kNearbyGridNum(5),
                  kRowNum(121), kColNum(121), kLevelNum(12),
                  kMinAddPointNumSmall(60), kMinAddFrontierPointNum(30),
                  kCellExploringToCoveredThr(1), kCellUnknownToExploringThr(1),
                  cur_robot_cell_ind_(-1) {}

    void Init(int rows, int cols, int levels, double cell_size, double cell_height, int nearby,
              int min_add_small, int min_add_frontier, int exp_to_cov, int unk_to_exp) {
        kRowNum = rows; kColNum = cols; kLevelNum = levels;
        kCellSize = cell_size; kCellHeight = cell_height; kNearbyGridNum = nearby;
        kMinAddPointNumSmall = min_add_small;
        kMinAddFrontierPointNum = min_add_frontier;
        kCellExploringToCoveredThr = exp_to_cov;
        kCellUnknownToExploringThr = unk_to_exp;

        Vec3d origin(-kRowNum * kCellSize / 2, -kColNum * kCellSize / 2, -kLevelNum * kCellHeight / 2);
        Vec3d res(kCellSize, kCellSize, kCellHeight);
        grid_.Resize(kRowNum, kColNum, kLevelNum, GridCell{}, origin, res);
        // Initialize cell centers
        for (int ind = 0; ind < grid_.CellNumber(); ind++) {
            Vec3d pos = grid_.Ind2Pos(ind);
            grid_.At(ind).center = {pos.x(), pos.y(), pos.z()};
        }
        initialized_ = true;
    }

    bool Initialized() const { return initialized_; }
    bool NeighborsInitialized() const { return neighbors_init_; }

    void UpdateRobotPosition(const Point3& pos) {
        robot_position_ = pos;
        Vec3i sub = grid_.Pos2Sub(Vec3d(pos.x, pos.y, pos.z));
        if (grid_.InRange(sub)) {
            cur_robot_cell_ind_ = grid_.Sub2Ind(sub);
        }
    }

    void UpdateNeighborCells(const Point3& pos) {
        // Re-center the grid on the robot position
        Vec3d robot_pos(pos.x, pos.y, pos.z);
        Vec3d origin(robot_pos.x() - kRowNum * kCellSize / 2.0,
                     robot_pos.y() - kColNum * kCellSize / 2.0,
                     robot_pos.z() - kLevelNum * kCellHeight / 2.0);
        grid_.SetOrigin(origin);

        neighbor_indices_.clear();
        Vec3i center = grid_.Pos2Sub(robot_pos);
        for (int dx = -kNearbyGridNum; dx <= kNearbyGridNum; dx++) {
            for (int dy = -kNearbyGridNum; dy <= kNearbyGridNum; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    Vec3i sub(center.x()+dx, center.y()+dy, center.z()+dz);
                    if (grid_.InRange(sub)) {
                        neighbor_indices_.push_back(grid_.Sub2Ind(sub));
                    }
                }
            }
        }
        neighbors_init_ = true;
    }

    // Update cell status using frontier points to drive UNSEEN → EXPLORING.
    // frontier_cloud contains detected frontier (unexplored boundary) points.
    void UpdateCellStatus(const std::vector<PointXYZI>& frontier_cloud) {
        // Count frontier points per neighbor cell
        std::map<int, int> frontier_count_per_cell;
        for (const auto& fp : frontier_cloud) {
            Vec3i sub = grid_.Pos2Sub(Vec3d(fp.x, fp.y, fp.z));
            if (grid_.InRange(sub)) {
                int ind = grid_.Sub2Ind(sub);
                frontier_count_per_cell[ind]++;
            }
        }

        int exploring_count = 0;
        for (int ind : neighbor_indices_) {
            auto& cell = grid_.At(ind);
            int fc = 0;
            auto it = frontier_count_per_cell.find(ind);
            if (it != frontier_count_per_cell.end()) fc = it->second;

            // Cells with enough frontier points transition to EXPLORING
            if (cell.status == CellStatus::UNSEEN && fc >= kCellUnknownToExploringThr) {
                cell.status = CellStatus::EXPLORING;
            }
            // Exploring cells with no remaining frontiers transition to COVERED
            if (cell.status == CellStatus::EXPLORING && fc == 0) {
                cell.visit_count++;
                if (cell.visit_count >= kCellExploringToCoveredThr * 10) {
                    cell.status = CellStatus::COVERED;
                }
            }
            if (cell.status == CellStatus::EXPLORING) exploring_count++;
        }
        return_home_ = (exploring_count == 0 && initialized_);
    }

    bool IsReturningHome() const { return return_home_; }
    int ExploringCount() const {
        int c = 0;
        for (int ind : neighbor_indices_)
            if (grid_.At(ind).status == CellStatus::EXPLORING) c++;
        return c;
    }
    void SetHomePosition(const Vec3d& pos) { home_position_ = pos; set_home_ = true; }
    bool HomeSet() const { return set_home_; }

    // Simple global TSP: visit exploring cells in nearest-first order
    ExplorationPath SolveGlobalTSP(const KeyposeGraph& keypose_graph) {
        ExplorationPath path;
        std::vector<int> exploring_cells;
        for (int ind : neighbor_indices_) {
            if (grid_.At(ind).status == CellStatus::EXPLORING)
                exploring_cells.push_back(ind);
        }
        if (exploring_cells.empty()) {
            if (set_home_) {
                PathNode rn; rn.position = Vec3d(robot_position_.x, robot_position_.y, robot_position_.z);
                rn.type = NodeType::ROBOT;
                path.Append(rn);
                PathNode hn; hn.position = home_position_; hn.type = NodeType::HOME;
                path.Append(hn);
            }
            return path;
        }

        // Build distance matrix for exploring cells + robot
        int n = (int)exploring_cells.size() + 1; // last is robot
        std::vector<std::vector<int>> dist_matrix(n, std::vector<int>(n, 0));
        std::vector<Point3> positions(n);
        for (int i = 0; i < (int)exploring_cells.size(); i++) {
            positions[i] = grid_.At(exploring_cells[i]).center;
        }
        positions[n-1] = robot_position_;
        for (int i = 0; i < n; i++) {
            for (int j = i+1; j < n; j++) {
                int d = (int)(10.0 * point_dist(positions[i], positions[j]));
                dist_matrix[i][j] = d;
                dist_matrix[j][i] = d;
            }
        }

        std::vector<int> order;
        solve_tsp(dist_matrix, n-1, order);

        PathNode robot_node;
        robot_node.position = Vec3d(robot_position_.x, robot_position_.y, robot_position_.z);
        robot_node.type = NodeType::ROBOT;
        path.Append(robot_node);

        for (int idx : order) {
            if (idx == n-1) continue; // skip robot
            PathNode node;
            const auto& c = positions[idx];
            node.position = Vec3d(c.x, c.y, c.z);
            node.type = NodeType::GLOBAL_VIEWPOINT;
            node.global_subspace_index = exploring_cells[idx];
            path.Append(node);
        }

        // Append home
        if (set_home_) {
            PathNode hn; hn.position = home_position_; hn.type = NodeType::HOME;
            path.Append(hn);
        }
        return path;
    }

    const std::vector<int>& GetNeighborIndices() const { return neighbor_indices_; }

private:
    bool initialized_;
    bool neighbors_init_;
    bool return_home_;
    bool set_home_;
    Vec3d home_position_;
    Point3 robot_position_;
    double kCellSize, kCellHeight;
    int kNearbyGridNum;
    int kRowNum, kColNum, kLevelNum;
    int kMinAddPointNumSmall, kMinAddFrontierPointNum;
    int kCellExploringToCoveredThr, kCellUnknownToExploringThr;
    int cur_robot_cell_ind_;
    Grid3D<GridCell> grid_;
    std::vector<int> neighbor_indices_;
};

// ============================================================================
// TARE Planner - main exploration planner class
// ============================================================================
class TarePlanner {
public:
    // --- Configuration ---
    bool kAutoStart = true;
    bool kRushHome = true;
    bool kUseTerrainHeight = false;
    bool kCheckTerrainCollision = true;
    bool kExtendWayPoint = true;
    bool kUseLineOfSightLookAheadPoint = true;
    bool kNoExplorationReturnHome = true;
    bool kUseMomentum = false;
    bool kUseFrontier = true;

    double kKeyposeCloudDwzFilterLeafSize = 0.2;
    double kRushHomeDist = 10.0;
    double kAtHomeDistThreshold = 0.5;
    double kTerrainCollisionThreshold = 0.5;
    double kLookAheadDistance = 5.0;
    double kExtendWayPointDistanceBig = 8.0;
    double kExtendWayPointDistanceSmall = 3.0;
    double kSensorRange = 10.0;

    int kDirectionChangeCounterThr = 4;
    int kDirectionNoChangeCounterThr = 5;

    // Planning env
    double kSurfaceCloudDwzLeafSize = 0.2;
    double kPointCloudCellSize = 24.0;
    double kPointCloudCellHeight = 3.0;
    int kPointCloudManagerNeighborCellNum = 5;
    double kFrontierClusterTolerance = 1.0;
    int kFrontierClusterMinSize = 30;

    // Rolling occupancy grid
    double kOccGridResX = 0.3;
    double kOccGridResY = 0.3;
    double kOccGridResZ = 0.3;

    // Grid world
    int kGridWorldXNum = 121, kGridWorldYNum = 121, kGridWorldZNum = 12;
    double kGridWorldCellHeight = 8.0;
    int kGridWorldNearbyGridNum = 5;
    int kMinAddPointNumSmall = 60, kMinAddFrontierPointNum = 30;
    int kCellExploringToCoveredThr = 1, kCellUnknownToExploringThr = 1;

    // Keypose graph
    double kKeyposeAddNodeMinDist = 0.5;
    double kKeyposeAddEdgeConnectDistThr = 0.5;
    double kKeyposeAddEdgeToLastKeyposeDistThr = 0.5;
    double kKeyposeAddEdgeVerticalThreshold = 0.5;

    // Viewpoint manager
    int kViewpointNumX = 80, kViewpointNumY = 80, kViewpointNumZ = 40;
    double kViewpointResX = 0.5, kViewpointResY = 0.5, kViewpointResZ = 0.5;
    double kNeighborRange = 3.0;

    // Update rate
    double kUpdateRate = 1.0; // Hz

    // --- State ---
    Point3 robot_position_;
    Point3 last_robot_position_;
    double robot_yaw_ = 0;
    bool initialized_ = false;
    bool exploration_finished_ = false;
    bool near_home_ = false;
    bool at_home_ = false;
    bool stopped_ = false;
    bool keypose_cloud_update_ = false;
    bool lookahead_point_update_ = false;
    bool start_exploration_ = false;
    bool lookahead_point_in_line_of_sight_ = true;
    Vec3d initial_position_ = Vec3d::Zero();
    Vec3d lookahead_point_ = Vec3d::Zero();
    Vec3d lookahead_point_direction_ = Vec3d(1,0,0);
    Vec3d moving_direction_ = Vec3d(1,0,0);
    int registered_cloud_count_ = 0;
    int keypose_count_ = 0;
    int direction_change_count_ = 0;
    int direction_no_change_count_ = 0;
    bool use_momentum_ = false;
    ExplorationPath exploration_path_;
    std::vector<Vec3d> visited_positions_;
    int cur_keypose_node_ind_ = 0;

    // Point cloud accumulation
    std::vector<PointXYZI> registered_scan_stack_;
    std::vector<PointXYZI> keypose_cloud_;

    // Frontier cloud
    std::vector<PointXYZI> frontier_cloud_;
    std::vector<PointXYZI> filtered_frontier_cloud_;

    double start_time_ = 0;

    // Sub-systems
    RollingOccupancyGrid rolling_occ_grid_;
    KeyposeGraph keypose_graph_;
    GridWorld grid_world_;

    std::mutex scan_mutex_;
    std::mutex odom_mutex_;

    // Incoming messages
    bool has_new_scan_ = false;
    std::vector<PointXYZI> latest_scan_;
    bool has_new_odom_ = false;
    Point3 latest_odom_pos_;
    double latest_odom_yaw_ = 0;

    // --- Initialization ---
    void Init() {
        keypose_graph_.kAddNodeMinDist = kKeyposeAddNodeMinDist;
        keypose_graph_.kAddNonKeyposeNodeMinDist = kKeyposeAddNodeMinDist;
        keypose_graph_.kAddEdgeConnectDistThr = kKeyposeAddEdgeConnectDistThr;
        keypose_graph_.kAddEdgeToLastKeyposeDistThr = kKeyposeAddEdgeToLastKeyposeDistThr;
        keypose_graph_.kAddEdgeVerticalThreshold = kKeyposeAddEdgeVerticalThreshold;

        rolling_occ_grid_.Init(kPointCloudCellSize, kPointCloudCellHeight,
                               kPointCloudManagerNeighborCellNum,
                               kOccGridResX, kOccGridResY, kOccGridResZ);

        grid_world_.Init(kGridWorldXNum, kGridWorldYNum, kGridWorldZNum,
                         kPointCloudCellSize, kGridWorldCellHeight, kGridWorldNearbyGridNum,
                         kMinAddPointNumSmall, kMinAddFrontierPointNum,
                         kCellExploringToCoveredThr, kCellUnknownToExploringThr);
    }

    // --- Callbacks ---
    void OnRegisteredScan(const std::vector<PointXYZI>& cloud) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = cloud;
        has_new_scan_ = true;
    }

    void OnOdometry(const Point3& pos, double yaw) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_pos_ = pos;
        latest_odom_yaw_ = yaw;
        has_new_odom_ = true;
    }

    // --- Compute waypoint output ---
    bool ComputeWaypoint(Point3& waypoint_out) {
        // Copy incoming data
        std::vector<PointXYZI> scan_copy;
        Point3 odom_pos;
        double odom_yaw;
        bool new_scan = false;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (has_new_odom_) {
                odom_pos = latest_odom_pos_;
                odom_yaw = latest_odom_yaw_;
                has_new_odom_ = false;
            } else {
                return false;
            }
        }
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if (has_new_scan_) {
                scan_copy = latest_scan_;
                has_new_scan_ = false;
                new_scan = true;
            }
        }

        // Update robot pose
        robot_position_ = odom_pos;
        robot_yaw_ = odom_yaw;

        // Record initial position
        if (std::abs(initial_position_.x()) < 0.01 &&
            std::abs(initial_position_.y()) < 0.01 &&
            std::abs(initial_position_.z()) < 0.01) {
            initial_position_ = Vec3d(robot_position_.x, robot_position_.y, robot_position_.z);
        }

        if (!kAutoStart && !start_exploration_) return false;

        if (!initialized_) {
            // Send initial waypoint ahead
            double lx = 12.0, ly = 0.0;
            double dx = cos(robot_yaw_) * lx - sin(robot_yaw_) * ly;
            double dy = sin(robot_yaw_) * lx + cos(robot_yaw_) * ly;
            waypoint_out.x = robot_position_.x + dx;
            waypoint_out.y = robot_position_.y + dy;
            waypoint_out.z = robot_position_.z;
            start_time_ = now_seconds();
            initialized_ = true;
            return true;
        }

        if (!new_scan) return false;

        // Process registered scan
        ProcessRegisteredScan(scan_copy);

        if (!keypose_cloud_update_) return false;
        keypose_cloud_update_ = false;

        Timer overall_timer("overall");
        overall_timer.Start();

        // Count direction changes
        CountDirectionChange();

        // Update rolling occupancy grid position
        Vec3d robot_pos_vec(robot_position_.x, robot_position_.y, robot_position_.z);
        rolling_occ_grid_.InitializeOrigin(robot_pos_vec - Vec3d(kPointCloudCellSize * kPointCloudManagerNeighborCellNum / 2,
                                                                   kPointCloudCellSize * kPointCloudManagerNeighborCellNum / 2,
                                                                   kPointCloudCellHeight * kPointCloudManagerNeighborCellNum / 2));
        rolling_occ_grid_.UpdateRobotPosition(robot_pos_vec);

        // Update grid world
        if (!grid_world_.NeighborsInitialized()) {
            grid_world_.UpdateNeighborCells(robot_position_);
        }
        grid_world_.UpdateRobotPosition(robot_position_);
        if (!grid_world_.HomeSet()) {
            grid_world_.SetHomePosition(initial_position_);
        }

        // Add keypose node
        cur_keypose_node_ind_ = keypose_graph_.AddKeyposeNode(robot_position_, keypose_count_);
        keypose_graph_.CheckConnectivity();

        // Update frontiers from rolling occupancy grid
        if (kUseFrontier) {
            double half_range = kViewpointNumX * kViewpointResX / 2 + kSensorRange * 2;
            Vec3d frontier_range(half_range, half_range, 2.0);
            rolling_occ_grid_.GetFrontier(frontier_cloud_, robot_pos_vec, frontier_range);

            // Simple cluster filtering (remove very small clusters)
            filtered_frontier_cloud_.clear();
            if ((int)frontier_cloud_.size() >= kFrontierClusterMinSize) {
                filtered_frontier_cloud_ = frontier_cloud_;
            }
        }

        // Update cell status using frontier points
        grid_world_.UpdateCellStatus(filtered_frontier_cloud_);

        // Global planning - TSP over exploring cells
        ExplorationPath global_path = grid_world_.SolveGlobalTSP(keypose_graph_);

        // Local planning - greedy coverage of nearby viewpoints
        ExplorationPath local_path = LocalPlanning(global_path);

        // Check exploration completion
        double robot_to_home = (robot_pos_vec - initial_position_).norm();
        near_home_ = robot_to_home < kRushHomeDist;
        at_home_ = robot_to_home < kAtHomeDistThreshold;

        double current_time = now_seconds();
        if (grid_world_.IsReturningHome() && (current_time - start_time_) > 5) {
            if (!exploration_finished_) {
                printf("[tare_planner] Exploration completed, returning home\n"); fflush(stdout);
            }
            exploration_finished_ = true;
        }

        if (exploration_finished_ && at_home_ && !stopped_) {
            printf("[tare_planner] Return home completed\n"); fflush(stdout);
            stopped_ = true;
        }

        // Concatenate path
        exploration_path_ = ConcatenateGlobalLocalPath(global_path, local_path);

        // Get look-ahead point
        lookahead_point_update_ = GetLookAheadPoint(exploration_path_, global_path, lookahead_point_);

        // Compute waypoint to publish
        ComputeWaypointFromLookahead(waypoint_out);

        // Debug: periodic status
        static int debug_counter = 0;
        if (++debug_counter % 5 == 0) {
            printf("[tare_planner] scan=%zu frontiers=%zu/%zu exploring=%s "
                   "gpath=%zu lpath=%zu wp=(%.1f,%.1f) robot=(%.1f,%.1f) "
                   "lookahead_ok=%d returning=%d finished=%d\n",
                   scan_copy.size(),
                   frontier_cloud_.size(), filtered_frontier_cloud_.size(),
                   grid_world_.IsReturningHome() ? "0(returning)" :
                       (grid_world_.ExploringCount() > 0 ?
                        (std::to_string(grid_world_.ExploringCount())).c_str() : "0"),
                   global_path.nodes.size(), local_path.nodes.size(),
                   waypoint_out.x, waypoint_out.y,
                   robot_position_.x, robot_position_.y,
                   lookahead_point_update_ ? 1 : 0,
                   grid_world_.IsReturningHome() ? 1 : 0,
                   exploration_finished_ ? 1 : 0);
            fflush(stdout);
        }

        last_robot_position_ = robot_position_;

        overall_timer.Stop();

        return true;
    }

private:
    void ProcessRegisteredScan(const std::vector<PointXYZI>& scan) {
        if (scan.empty()) return;

        // Accumulate
        for (const auto& p : scan) registered_scan_stack_.push_back(p);

        // Downsample the incoming scan and update occupancy
        std::vector<PointXYZI> scan_dwz = scan;
        float leaf = (float)kKeyposeCloudDwzFilterLeafSize;
        downsample_cloud(scan_dwz, leaf, leaf, leaf);

        // Feed rolling occupancy grid
        rolling_occ_grid_.UpdateOccupancy(scan_dwz);
        rolling_occ_grid_.RayTrace(Vec3d(robot_position_.x, robot_position_.y, robot_position_.z));

        registered_cloud_count_ = (registered_cloud_count_ + 1) % 5;
        if (registered_cloud_count_ == 0) {
            keypose_count_++;

            // Downsample accumulated scans
            downsample_cloud(registered_scan_stack_, leaf, leaf, leaf);
            keypose_cloud_ = registered_scan_stack_;
            registered_scan_stack_.clear();
            keypose_cloud_update_ = true;
        }
    }

    void CountDirectionChange() {
        Vec3d cur_dir(robot_position_.x - last_robot_position_.x,
                      robot_position_.y - last_robot_position_.y,
                      robot_position_.z - last_robot_position_.z);
        if (cur_dir.norm() > 0.5) {
            if (moving_direction_.dot(cur_dir) < 0) {
                direction_change_count_++;
                direction_no_change_count_ = 0;
                if (direction_change_count_ > kDirectionChangeCounterThr) {
                    use_momentum_ = true;
                }
            } else {
                direction_no_change_count_++;
                if (direction_no_change_count_ > kDirectionNoChangeCounterThr) {
                    direction_change_count_ = 0;
                    use_momentum_ = false;
                }
            }
            moving_direction_ = cur_dir;
        }
    }

    void UpdateVisitedPositions() {
        Vec3d cur(robot_position_.x, robot_position_.y, robot_position_.z);
        bool existing = false;
        for (const auto& vp : visited_positions_) {
            if ((cur - vp).norm() < 1.0) { existing = true; break; }
        }
        if (!existing) visited_positions_.push_back(cur);
    }

    ExplorationPath LocalPlanning(const ExplorationPath& global_path) {
        ExplorationPath local_path;

        // Simplified local coverage: find points along global path within
        // the local planning horizon and produce a simple path through them
        Vec3d robot_pos(robot_position_.x, robot_position_.y, robot_position_.z);
        double local_range = kViewpointNumX * kViewpointResX / 2;

        // Collect reachable global path nodes in local range
        std::vector<PathNode> local_nodes;
        PathNode robot_node;
        robot_node.position = robot_pos;
        robot_node.type = NodeType::ROBOT;
        local_nodes.push_back(robot_node);

        for (const auto& node : global_path.nodes) {
            if ((node.position - robot_pos).norm() < local_range &&
                node.type != NodeType::ROBOT) {
                local_nodes.push_back(node);
            }
        }

        // Also add frontier points as local viewpoints if they have
        // enough density
        if (!filtered_frontier_cloud_.empty()) {
            // Sample up to 10 frontier cluster centroids
            int step = std::max(1, (int)filtered_frontier_cloud_.size() / 10);
            for (int i = 0; i < (int)filtered_frontier_cloud_.size(); i += step) {
                const auto& p = filtered_frontier_cloud_[i];
                Vec3d fp(p.x, p.y, p.z);
                if ((fp - robot_pos).norm() < local_range) {
                    PathNode fn;
                    fn.position = fp;
                    fn.type = NodeType::LOCAL_VIEWPOINT;
                    local_nodes.push_back(fn);
                }
            }
        }

        if (local_nodes.size() <= 1) {
            // Just robot, add via point ahead
            double lx = 3.0;
            PathNode ahead;
            ahead.position = robot_pos + Vec3d(cos(robot_yaw_) * lx, sin(robot_yaw_) * lx, 0);
            ahead.type = NodeType::LOCAL_VIA_POINT;
            local_path.Append(robot_node);
            local_path.Append(ahead);
            return local_path;
        }

        // Build distance matrix for local TSP
        int n = (int)local_nodes.size();
        std::vector<std::vector<int>> dist_matrix(n, std::vector<int>(n, 0));
        for (int i = 0; i < n; i++) {
            for (int j = i+1; j < n; j++) {
                int d = (int)(10.0 * (local_nodes[i].position - local_nodes[j].position).norm());
                dist_matrix[i][j] = d;
                dist_matrix[j][i] = d;
            }
        }

        std::vector<int> order;
        solve_tsp(dist_matrix, 0, order); // depot=0 is robot

        for (int idx : order) {
            local_path.Append(local_nodes[idx]);
        }
        // Close the loop back to start
        if (!order.empty() && order.front() != order.back()) {
            local_path.Append(local_nodes[0]);
        }

        return local_path;
    }

    ExplorationPath ConcatenateGlobalLocalPath(const ExplorationPath& global_path,
                                                const ExplorationPath& local_path) {
        ExplorationPath full_path;
        if (exploration_finished_ && near_home_ && kRushHome) {
            PathNode rn;
            rn.position = Vec3d(robot_position_.x, robot_position_.y, robot_position_.z);
            rn.type = NodeType::ROBOT;
            full_path.Append(rn);
            PathNode hn;
            hn.position = initial_position_;
            hn.type = NodeType::HOME;
            full_path.Append(hn);
            return full_path;
        }

        double global_len = global_path.GetLength();
        double local_len = local_path.GetLength();
        if (global_len < 3 && local_len < 5) {
            return full_path;
        }

        full_path = local_path;
        if (!full_path.nodes.empty()) {
            // Ensure correct start/end types
            if (full_path.nodes.front().type == NodeType::LOCAL_PATH_END &&
                full_path.nodes.back().type == NodeType::LOCAL_PATH_START) {
                full_path.Reverse();
            }
        }
        return full_path;
    }

    bool GetLookAheadPoint(const ExplorationPath& local_path,
                            const ExplorationPath& global_path,
                            Vec3d& lookahead_point) {
        Vec3d robot_pos(robot_position_.x, robot_position_.y, robot_position_.z);
        if (local_path.GetNodeNum() < 2) {
            // Follow global path direction
            for (const auto& n : global_path.nodes) {
                if ((n.position - robot_pos).norm() > kLookAheadDistance / 2) {
                    lookahead_point = n.position;
                    return false;
                }
            }
            return false;
        }

        // Find robot index
        int robot_i = 0;
        for (int i = 0; i < (int)local_path.nodes.size(); i++) {
            if (local_path.nodes[i].type == NodeType::ROBOT) {
                robot_i = i;
                break;
            }
        }

        // Walk forward to find lookahead point
        double length_from_robot = 0;
        for (int i = robot_i + 1; i < (int)local_path.nodes.size(); i++) {
            length_from_robot += (local_path.nodes[i].position - local_path.nodes[i-1].position).norm();
            if (length_from_robot > kLookAheadDistance ||
                local_path.nodes[i].type == NodeType::LOCAL_VIEWPOINT ||
                local_path.nodes[i].type == NodeType::LOCAL_PATH_START ||
                local_path.nodes[i].type == NodeType::LOCAL_PATH_END ||
                local_path.nodes[i].type == NodeType::GLOBAL_VIEWPOINT ||
                i == (int)local_path.nodes.size() - 1) {
                lookahead_point = local_path.nodes[i].position;
                lookahead_point_direction_ = lookahead_point - robot_pos;
                lookahead_point_direction_.z() = 0;
                if (lookahead_point_direction_.norm() > 1e-6)
                    lookahead_point_direction_.normalize();
                return true;
            }
        }

        // Walk backward
        length_from_robot = 0;
        for (int i = robot_i - 1; i >= 0; i--) {
            length_from_robot += (local_path.nodes[i].position - local_path.nodes[i+1].position).norm();
            if (length_from_robot > kLookAheadDistance ||
                local_path.nodes[i].type == NodeType::LOCAL_VIEWPOINT ||
                i == 0) {
                lookahead_point = local_path.nodes[i].position;
                lookahead_point_direction_ = lookahead_point - robot_pos;
                lookahead_point_direction_.z() = 0;
                if (lookahead_point_direction_.norm() > 1e-6)
                    lookahead_point_direction_.normalize();
                return true;
            }
        }

        return false;
    }

    void ComputeWaypointFromLookahead(Point3& waypoint) {
        if (exploration_finished_ && near_home_ && kRushHome) {
            waypoint.x = initial_position_.x();
            waypoint.y = initial_position_.y();
            waypoint.z = initial_position_.z();
            return;
        }

        double dx = lookahead_point_.x() - robot_position_.x;
        double dy = lookahead_point_.y() - robot_position_.y;
        double r = sqrt(dx*dx + dy*dy);

        double extend_dist = lookahead_point_in_line_of_sight_
                             ? kExtendWayPointDistanceBig
                             : kExtendWayPointDistanceSmall;
        if (r < extend_dist && kExtendWayPoint && r > 1e-6) {
            dx = dx / r * extend_dist;
            dy = dy / r * extend_dist;
        }

        waypoint.x = dx + robot_position_.x;
        waypoint.y = dy + robot_position_.y;
        waypoint.z = lookahead_point_.z();
    }
};

// ============================================================================
// LCM Handlers
// ============================================================================
class Handlers {
public:
    TarePlanner* planner;

    void registeredScanHandler(const lcm::ReceiveBuffer*,
                               const std::string&,
                               const sensor_msgs::PointCloud2* msg) {
        auto points = smartnav::parse_pointcloud2(*msg);
        std::vector<PointXYZI> cloud;
        cloud.reserve(points.size());
        for (const auto& p : points) {
            cloud.push_back({p.x, p.y, p.z, p.intensity});
        }
        planner->OnRegisteredScan(cloud);
    }

    void odometryHandler(const lcm::ReceiveBuffer*,
                         const std::string&,
                         const nav_msgs::Odometry* msg) {
        Point3 pos;
        pos.x = msg->pose.pose.position.x;
        pos.y = msg->pose.pose.position.y;
        pos.z = msg->pose.pose.position.z;

        double roll, pitch, yaw;
        smartnav::quat_to_rpy(msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z,
                              msg->pose.pose.orientation.w,
                              roll, pitch, yaw);

        planner->OnOdometry(pos, yaw);
    }
};

// ============================================================================
// main
// ============================================================================
int main(int argc, char** argv)
{
    // --- Signal handling ---
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    // --- Parse CLI args ---
    dimos::NativeModule mod(argc, argv);

    TarePlanner planner;

    // General parameters
    planner.kAutoStart        = mod.arg_bool("kAutoStart", true);
    planner.kRushHome         = mod.arg_bool("kRushHome", true);
    planner.kUseTerrainHeight = mod.arg_bool("kUseTerrainHeight", false);
    planner.kCheckTerrainCollision = mod.arg_bool("kCheckTerrainCollision", true);
    planner.kExtendWayPoint   = mod.arg_bool("kExtendWayPoint", true);
    planner.kUseLineOfSightLookAheadPoint = mod.arg_bool("kUseLineOfSightLookAheadPoint", true);
    planner.kNoExplorationReturnHome = mod.arg_bool("kNoExplorationReturnHome", true);
    planner.kUseMomentum      = mod.arg_bool("kUseMomentum", false);
    planner.kUseFrontier      = mod.arg_bool("kUseFrontier", true);

    planner.kKeyposeCloudDwzFilterLeafSize = mod.arg_float("kKeyposeCloudDwzFilterLeafSize", 0.2f);
    planner.kRushHomeDist     = mod.arg_float("kRushHomeDist", 10.0f);
    planner.kAtHomeDistThreshold = mod.arg_float("kAtHomeDistThreshold", 0.5f);
    planner.kTerrainCollisionThreshold = mod.arg_float("kTerrainCollisionThreshold", 0.5f);
    planner.kLookAheadDistance = mod.arg_float("kLookAheadDistance", 5.0f);
    planner.kExtendWayPointDistanceBig = mod.arg_float("kExtendWayPointDistanceBig", 8.0f);
    planner.kExtendWayPointDistanceSmall = mod.arg_float("kExtendWayPointDistanceSmall", 3.0f);
    planner.kSensorRange      = mod.arg_float("kSensorRange", 10.0f);

    planner.kDirectionChangeCounterThr = mod.arg_int("kDirectionChangeCounterThr", 4);
    planner.kDirectionNoChangeCounterThr = mod.arg_int("kDirectionNoChangeCounterThr", 5);

    // Planning env parameters
    planner.kSurfaceCloudDwzLeafSize = mod.arg_float("kSurfaceCloudDwzLeafSize", 0.2f);
    planner.kPointCloudCellSize = mod.arg_float("kPointCloudCellSize", 24.0f);
    planner.kPointCloudCellHeight = mod.arg_float("kPointCloudCellHeight", 3.0f);
    planner.kPointCloudManagerNeighborCellNum = mod.arg_int("kPointCloudManagerNeighborCellNum", 5);
    planner.kFrontierClusterTolerance = mod.arg_float("kFrontierClusterTolerance", 1.0f);
    planner.kFrontierClusterMinSize = mod.arg_int("kFrontierClusterMinSize", 30);

    // Rolling occupancy grid
    planner.kOccGridResX = mod.arg_float("rolling_occupancy_grid_resolution_x", 0.3f);
    planner.kOccGridResY = mod.arg_float("rolling_occupancy_grid_resolution_y", 0.3f);
    planner.kOccGridResZ = mod.arg_float("rolling_occupancy_grid_resolution_z", 0.3f);

    // Grid world
    planner.kGridWorldXNum = mod.arg_int("kGridWorldXNum", 121);
    planner.kGridWorldYNum = mod.arg_int("kGridWorldYNum", 121);
    planner.kGridWorldZNum = mod.arg_int("kGridWorldZNum", 12);
    planner.kGridWorldCellHeight = mod.arg_float("kGridWorldCellHeight", 8.0f);
    planner.kGridWorldNearbyGridNum = mod.arg_int("kGridWorldNearbyGridNum", 5);
    planner.kMinAddPointNumSmall = mod.arg_int("kMinAddPointNumSmall", 60);
    planner.kMinAddFrontierPointNum = mod.arg_int("kMinAddFrontierPointNum", 30);
    planner.kCellExploringToCoveredThr = mod.arg_int("kCellExploringToCoveredThr", 1);
    planner.kCellUnknownToExploringThr = mod.arg_int("kCellUnknownToExploringThr", 1);

    // Keypose graph
    planner.kKeyposeAddNodeMinDist = mod.arg_float("keypose_graph_kAddNodeMinDist", 0.5f);
    planner.kKeyposeAddEdgeConnectDistThr = mod.arg_float("keypose_graph_kAddEdgeConnectDistThr", 0.5f);
    planner.kKeyposeAddEdgeToLastKeyposeDistThr = mod.arg_float("keypose_graph_kAddEdgeToLastKeyposeDistThr", 0.5f);
    planner.kKeyposeAddEdgeVerticalThreshold = mod.arg_float("keypose_graph_kAddEdgeVerticalThreshold", 0.5f);

    // Viewpoint manager
    planner.kViewpointNumX = mod.arg_int("viewpoint_manager_number_x", 80);
    planner.kViewpointNumY = mod.arg_int("viewpoint_manager_number_y", 80);
    planner.kViewpointNumZ = mod.arg_int("viewpoint_manager_number_z", 40);
    planner.kViewpointResX = mod.arg_float("viewpoint_manager_resolution_x", 0.5f);
    planner.kViewpointResY = mod.arg_float("viewpoint_manager_resolution_y", 0.5f);
    planner.kViewpointResZ = mod.arg_float("viewpoint_manager_resolution_z", 0.5f);
    planner.kNeighborRange = mod.arg_float("kNeighborRange", 3.0f);

    // Update rate
    planner.kUpdateRate = mod.arg_float("update_rate", 1.0f);

    // Initialize planner
    planner.Init();

    // --- Resolve LCM topics ---
    const std::string scan_topic = mod.topic("registered_scan");
    const std::string odom_topic = mod.topic("odometry");
    const std::string waypoint_topic = mod.topic("way_point");

    // --- Create LCM instance ---
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[tare_planner] ERROR: LCM init failed\n");
        return 1;
    }

    // --- Subscribe ---
    Handlers handlers;
    handlers.planner = &planner;
    lcm.subscribe(scan_topic, &Handlers::registeredScanHandler, &handlers);
    lcm.subscribe(odom_topic, &Handlers::odometryHandler, &handlers);

    printf("[tare_planner] Running. scan=%s  odom=%s  waypoint=%s\n",
           scan_topic.c_str(), odom_topic.c_str(), waypoint_topic.c_str());
    fflush(stdout);

    // --- Main loop ---
    int loop_period_ms = (int)(1000.0 / std::max(planner.kUpdateRate, 0.1));

    while (!g_shutdown.load()) {
        // Handle LCM with timeout
        int timeout_ms = std::min(loop_period_ms, 100);
        lcm.handleTimeout(timeout_ms);

        // Process at update rate
        Point3 waypoint;
        if (planner.ComputeWaypoint(waypoint)) {
            geometry_msgs::PointStamped wp_msg;
            wp_msg.header = dimos::make_header("map", now_seconds());
            wp_msg.point.x = waypoint.x;
            wp_msg.point.y = waypoint.y;
            wp_msg.point.z = waypoint.z;
            lcm.publish(waypoint_topic, &wp_msg);
        }
    }

    printf("[tare_planner] Shutting down.\n"); fflush(stdout);
    return 0;
}
