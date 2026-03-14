// AriseSLAM — dimos NativeModule port
// Ported from ROS2: src/slam/arise_slam_mid360
//
// LiDAR SLAM system with:
//   - Curvature-based feature extraction (edge + planar features)
//   - Scan-to-map matching via Ceres optimization
//   - IMU preintegration for motion prediction
//   - Rolling local map with KD-tree search
//   - Publishes registered scan (world-frame) and odometry
//
// Subscribes:  raw_points (PointCloud2), imu (Imu)
// Publishes:   registered_scan (PointCloud2), odometry (Odometry)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/Imu.hpp"
#include "nav_msgs/Odometry.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using Q4D = Eigen::Quaterniond;

static constexpr double DEG2RAD = M_PI / 180.0;
static constexpr double RAD2DEG = 180.0 / M_PI;

// ─── Configuration ───────────────────────────────────────────────────────────

struct SLAMConfig {
    // Feature extraction
    double edge_threshold       = 1.0;    // Curvature threshold for edge features
    double surf_threshold       = 0.1;    // Curvature threshold for planar features
    int    edge_feature_min     = 10;     // Min valid edge features
    int    surf_feature_min     = 100;    // Min valid surface features
    double scan_voxel_size      = 0.1;    // Input cloud downsampling

    // Local map
    int    map_grid_width       = 21;     // Grid cells per axis (X/Y)
    int    map_grid_depth       = 11;     // Grid cells (Z)
    double map_voxel_res        = 50.0;   // Meters per grid cell
    float  line_res             = 0.2f;   // Edge feature downsample resolution
    float  plane_res            = 0.4f;   // Planar feature downsample resolution

    // Scan matching
    int    max_icp_iterations   = 4;      // Outer ICP iterations
    int    max_lm_iterations    = 15;     // Ceres LM iterations per ICP step
    int    edge_nbr_neighbors   = 5;      // KNN for edge matching
    int    surf_nbr_neighbors   = 5;      // KNN for surface matching
    double max_edge_distance    = 1.0;    // Max distance for edge correspondences
    double max_surf_distance    = 1.0;    // Max distance for surface correspondences

    // IMU
    bool   use_imu              = true;   // Use IMU for prediction
    double imu_acc_noise        = 0.01;
    double imu_gyr_noise        = 0.001;
    double gravity              = 9.80511;

    // Output
    double min_publish_interval = 0.05;   // Min time between odometry publishes
    bool   publish_map          = false;  // Publish local map periodically
    double map_publish_rate     = 0.2;    // Map publish rate (Hz)
    double map_viz_voxel_size   = 0.2;    // Visualization map voxel size

    // Initialization
    double init_x = 0.0, init_y = 0.0, init_z = 0.0;
    double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;

    // Sensor config
    int    n_scan = 6;            // Number of scan lines (Mid-360 ≈ 6)
    double blind_distance = 0.5;  // Min range to filter near points
    double max_range = 100.0;     // Max range
};

// ─── Ceres SE3 Manifold ─────────────────────────────────────────────────────
// Pose parameterization: [tx, ty, tz, qx, qy, qz, qw]
// Local perturbation in tangent space (6-DOF)

class PoseSE3Manifold : public ceres::Manifold {
public:
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }

    // Quaternion multiply in [x,y,z,w] storage order: result = a * b
    static void quatMul_xyzw(const double* a, const double* b, double* out) {
        // a = [ax, ay, az, aw], b = [bx, by, bz, bw]
        double aw = a[3], ax = a[0], ay = a[1], az = a[2];
        double bw = b[3], bx = b[0], by = b[1], bz = b[2];
        out[0] = aw*bx + ax*bw + ay*bz - az*by;  // x
        out[1] = aw*by - ax*bz + ay*bw + az*bx;  // y
        out[2] = aw*bz + ax*by - ay*bx + az*bw;  // z
        out[3] = aw*bw - ax*bx - ay*by - az*bz;  // w
    }

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
        // Translation update
        x_plus_delta[0] = x[0] + delta[0];
        x_plus_delta[1] = x[1] + delta[1];
        x_plus_delta[2] = x[2] + delta[2];

        // Rotation update: q_new = q_old * exp(delta_rot)
        // dq stored as [qx, qy, qz, qw]
        double dq[4];
        double half_theta[3] = {delta[3] * 0.5, delta[4] * 0.5, delta[5] * 0.5};
        double theta_sq = half_theta[0]*half_theta[0] + half_theta[1]*half_theta[1]
                        + half_theta[2]*half_theta[2];
        if (theta_sq > 0.0) {
            double theta = std::sqrt(theta_sq);
            double k = std::sin(theta) / theta;
            dq[0] = k * half_theta[0];  // qx
            dq[1] = k * half_theta[1];  // qy
            dq[2] = k * half_theta[2];  // qz
            dq[3] = std::cos(theta);    // qw
        } else {
            dq[0] = half_theta[0];
            dq[1] = half_theta[1];
            dq[2] = half_theta[2];
            dq[3] = 1.0;
        }

        // Quaternion multiplication: q_old * dq (both in [x,y,z,w] order)
        quatMul_xyzw(x + 3, dq, x_plus_delta + 3);

        return true;
    }

    bool PlusJacobian(const double* x, double* jacobian) const override {
        // 7x6 Jacobian
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
        J.setZero();
        J(0, 0) = 1.0; J(1, 1) = 1.0; J(2, 2) = 1.0;
        // Quaternion part: simplified for small perturbation
        J(3, 3) = 0.5; J(4, 4) = 0.5; J(5, 5) = 0.5;
        J(6, 3) = 0.0; J(6, 4) = 0.0; J(6, 5) = 0.0;
        (void)x;
        return true;
    }

    bool Minus(const double* y, const double* x, double* y_minus_x) const override {
        y_minus_x[0] = y[0] - x[0];
        y_minus_x[1] = y[1] - x[1];
        y_minus_x[2] = y[2] - x[2];

        // Log of relative quaternion: q_rel = x_inv * y
        // x_inv in [x,y,z,w] order: conjugate = [-x, -y, -z, w]
        double x_inv[4] = {-x[3], -x[4], -x[5], x[6]};
        double q_rel[4];
        quatMul_xyzw(x_inv, y + 3, q_rel);

        double sin_sq = q_rel[0]*q_rel[0] + q_rel[1]*q_rel[1] + q_rel[2]*q_rel[2];
        if (sin_sq > 1e-10) {
            double sin_val = std::sqrt(sin_sq);
            double theta = 2.0 * std::atan2(sin_val, q_rel[3]);
            double k = theta / sin_val;
            y_minus_x[3] = k * q_rel[0];
            y_minus_x[4] = k * q_rel[1];
            y_minus_x[5] = k * q_rel[2];
        } else {
            y_minus_x[3] = 2.0 * q_rel[0];
            y_minus_x[4] = 2.0 * q_rel[1];
            y_minus_x[5] = 2.0 * q_rel[2];
        }
        return true;
    }

    bool MinusJacobian(const double* x, double* jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobian);
        J.setZero();
        J(0, 0) = 1.0; J(1, 1) = 1.0; J(2, 2) = 1.0;
        J(3, 3) = 2.0; J(4, 4) = 2.0; J(5, 5) = 2.0;
        (void)x;
        return true;
    }
};

// ─── Ceres Cost Functions ────────────────────────────────────────────────────
// Port of ceresCostFunction.h and lidarOptimization.h

// Edge cost: point-to-line distance
// Parameters: [tx, ty, tz, qx, qy, qz, qw]
// Residual: cross product distance from current point to line (lp_a, lp_b)
struct EdgeCostFunction : public ceres::SizedCostFunction<3, 7> {
    V3D curr_point;   // Point in body frame
    V3D last_point_a; // Line point A in map frame
    V3D last_point_b; // Line point B in map frame

    EdgeCostFunction(const V3D& cp, const V3D& lpa, const V3D& lpb)
        : curr_point(cp), last_point_a(lpa), last_point_b(lpb) {}

    bool Evaluate(double const* const* parameters,
                  double* residuals, double** jacobians) const override {
        const double* p = parameters[0];
        V3D t(p[0], p[1], p[2]);
        Q4D q(p[6], p[3], p[4], p[5]);  // Ceres: [qx,qy,qz,qw] storage, but Q4D(w,x,y,z)
        q.normalize();

        V3D lp = q * curr_point + t;  // Transform point to map frame

        V3D nu = (lp - last_point_a).cross(lp - last_point_b);
        V3D de = last_point_a - last_point_b;
        double de_norm = de.norm();
        if (de_norm < 1e-10) de_norm = 1e-10;

        residuals[0] = nu.x() / de_norm;
        residuals[1] = nu.y() / de_norm;
        residuals[2] = nu.z() / de_norm;

        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();

            // d(residual)/d(translation) = d(lp)/dt cross stuff / de_norm
            // lp = q*cp + t, so d(lp)/dt = I
            // d(nu)/d(lp) is the skew-symmetric cross-product matrix
            V3D da = lp - last_point_a;
            V3D db = lp - last_point_b;

            // d(cross(da, db))/d(lp) = skew(db) - skew(da)
            // Since d(da)/d(lp) = I and d(db)/d(lp) = I
            Eigen::Matrix3d skew_da, skew_db;
            skew_da << 0, -da.z(), da.y(), da.z(), 0, -da.x(), -da.y(), da.x(), 0;
            skew_db << 0, -db.z(), db.y(), db.z(), 0, -db.x(), -db.y(), db.x(), 0;

            Eigen::Matrix3d d_nu_d_lp = skew_db - skew_da;
            Eigen::Matrix3d d_res_d_lp = d_nu_d_lp / de_norm;

            // Translation jacobian: d(res)/d(t) = d(res)/d(lp) * d(lp)/d(t) = d(res)/d(lp) * I
            J.block<3,3>(0,0) = d_res_d_lp;

            // Rotation jacobian: d(res)/d(delta_theta) = d(res)/d(lp) * d(lp)/d(delta_theta)
            // d(lp)/d(delta_theta) = -[q*cp]_x  (skew of rotated point)
            V3D qcp = q * curr_point;
            Eigen::Matrix3d skew_qcp;
            skew_qcp << 0, -qcp.z(), qcp.y(), qcp.z(), 0, -qcp.x(), -qcp.y(), qcp.x(), 0;
            J.block<3,3>(0,3) = -d_res_d_lp * skew_qcp;
            // qw jacobian (column 6) stays zero — handled by manifold
        }
        return true;
    }
};

// Surface cost: point-to-plane distance
// Residual: (lp - plane_center) . normal
struct SurfCostFunction : public ceres::SizedCostFunction<1, 7> {
    V3D curr_point;    // Point in body frame
    V3D plane_normal;  // Plane normal in map frame
    double d_offset;   // Plane offset (normal . plane_point)

    SurfCostFunction(const V3D& cp, const V3D& normal, double d)
        : curr_point(cp), plane_normal(normal), d_offset(d) {}

    bool Evaluate(double const* const* parameters,
                  double* residuals, double** jacobians) const override {
        const double* p = parameters[0];
        V3D t(p[0], p[1], p[2]);
        Q4D q(p[6], p[3], p[4], p[5]);
        q.normalize();

        V3D lp = q * curr_point + t;
        residuals[0] = plane_normal.dot(lp) - d_offset;

        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();

            // Translation jacobian
            J(0, 0) = plane_normal.x();
            J(0, 1) = plane_normal.y();
            J(0, 2) = plane_normal.z();

            // Rotation jacobian: d(n.lp)/d(delta_theta) = n^T * (-[q*cp]_x)
            V3D qcp = q * curr_point;
            J(0, 3) = -(plane_normal.y() * qcp.z() - plane_normal.z() * qcp.y());
            J(0, 4) = -(plane_normal.z() * qcp.x() - plane_normal.x() * qcp.z());
            J(0, 5) = -(plane_normal.x() * qcp.y() - plane_normal.y() * qcp.x());
        }
        return true;
    }
};

// ─── Feature Extraction ──────────────────────────────────────────────────────
// Port of featureExtraction.cpp — curvature-based edge/planar classification

struct FeatureSet {
    CloudType::Ptr edges;
    CloudType::Ptr planes;
};

FeatureSet extractFeatures(const CloudType::Ptr& cloud_in,
                           const SLAMConfig& config) {
    FeatureSet features;
    features.edges.reset(new CloudType);
    features.planes.reset(new CloudType);

    if (cloud_in->empty()) return features;

    int cloud_size = static_cast<int>(cloud_in->size());
    if (cloud_size < 20) return features;

    // Compute curvature for each point
    std::vector<double> curvatures(cloud_size, 0.0);
    std::vector<bool> picked(cloud_size, false);

    // Neighborhood size for curvature computation
    const int half_window = 5;

    for (int i = half_window; i < cloud_size - half_window; ++i) {
        double diff_x = 0, diff_y = 0, diff_z = 0;
        for (int j = -half_window; j <= half_window; ++j) {
            if (j == 0) continue;
            diff_x += cloud_in->points[i + j].x - cloud_in->points[i].x;
            diff_y += cloud_in->points[i + j].y - cloud_in->points[i].y;
            diff_z += cloud_in->points[i + j].z - cloud_in->points[i].z;
        }
        curvatures[i] = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
    }

    // Segment cloud into regions and extract features
    // Process in segments to get spatially distributed features
    int n_segments = 6;
    int segment_size = (cloud_size - 2 * half_window) / n_segments;

    for (int seg = 0; seg < n_segments; ++seg) {
        int start = half_window + seg * segment_size;
        int end = (seg == n_segments - 1) ? (cloud_size - half_window) : (start + segment_size);

        // Sort indices by curvature within segment
        std::vector<int> indices(end - start);
        std::iota(indices.begin(), indices.end(), start);
        std::sort(indices.begin(), indices.end(), [&](int a, int b) {
            return curvatures[a] > curvatures[b];
        });

        // Extract edge features (high curvature)
        int edge_count = 0;
        for (int idx : indices) {
            if (picked[idx]) continue;
            if (curvatures[idx] < config.edge_threshold) break;

            edge_count++;
            if (edge_count > 20) break;

            features.edges->push_back(cloud_in->points[idx]);
            picked[idx] = true;

            // Mark neighbors as picked to avoid clustering
            for (int j = -half_window; j <= half_window; ++j) {
                int ni = idx + j;
                if (ni >= 0 && ni < cloud_size) picked[ni] = true;
            }
        }

        // Extract planar features (low curvature)
        int plane_count = 0;
        for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
            int idx = *it;
            if (picked[idx]) continue;
            if (curvatures[idx] > config.surf_threshold) break;

            plane_count++;
            if (plane_count > 40) break;

            features.planes->push_back(cloud_in->points[idx]);
            picked[idx] = true;
        }
    }

    return features;
}

// ─── Local Map ───────────────────────────────────────────────────────────────
// Simplified rolling grid map for edge and planar features

class LocalMap {
public:
    CloudType::Ptr edge_map;
    CloudType::Ptr surf_map;
    pcl::KdTreeFLANN<PointType> edge_kdtree;
    pcl::KdTreeFLANN<PointType> surf_kdtree;
    bool edge_tree_valid = false;
    bool surf_tree_valid = false;

    V3D origin = V3D::Zero();
    double max_range;
    float line_res;
    float plane_res;

    LocalMap(double range = 100.0, float lr = 0.2f, float pr = 0.4f)
        : max_range(range), line_res(lr), plane_res(pr) {
        edge_map.reset(new CloudType);
        surf_map.reset(new CloudType);
    }

    void addEdgeCloud(const CloudType::Ptr& cloud, const V3D& position) {
        *edge_map += *cloud;
        // Remove points too far from current position
        cropCloud(edge_map, position, max_range);
        // Downsample
        if (line_res > 0 && edge_map->size() > 0) {
            pcl::VoxelGrid<PointType> vg;
            vg.setLeafSize(line_res, line_res, line_res);
            vg.setInputCloud(edge_map);
            vg.filter(*edge_map);
        }
        // Rebuild KD-tree
        if (edge_map->size() > 0) {
            edge_kdtree.setInputCloud(edge_map);
            edge_tree_valid = true;
        }
    }

    void addSurfCloud(const CloudType::Ptr& cloud, const V3D& position) {
        *surf_map += *cloud;
        cropCloud(surf_map, position, max_range);
        if (plane_res > 0 && surf_map->size() > 0) {
            pcl::VoxelGrid<PointType> vg;
            vg.setLeafSize(plane_res, plane_res, plane_res);
            vg.setInputCloud(surf_map);
            vg.filter(*surf_map);
        }
        if (surf_map->size() > 0) {
            surf_kdtree.setInputCloud(surf_map);
            surf_tree_valid = true;
        }
    }

    CloudType::Ptr getMapCloud(double voxel_size = 0.2) const {
        CloudType::Ptr combined(new CloudType);
        *combined += *edge_map;
        *combined += *surf_map;
        if (voxel_size > 0 && combined->size() > 0) {
            pcl::VoxelGrid<PointType> vg;
            vg.setLeafSize(voxel_size, voxel_size, voxel_size);
            vg.setInputCloud(combined);
            vg.filter(*combined);
        }
        return combined;
    }

private:
    void cropCloud(CloudType::Ptr& cloud, const V3D& center, double range) {
        CloudType::Ptr cropped(new CloudType);
        cropped->reserve(cloud->size());
        double range_sq = range * range;
        for (const auto& pt : *cloud) {
            double dx = pt.x - center.x();
            double dy = pt.y - center.y();
            double dz = pt.z - center.z();
            if (dx*dx + dy*dy + dz*dz < range_sq) {
                cropped->push_back(pt);
            }
        }
        cloud = cropped;
    }
};

// ─── IMU Integrator ──────────────────────────────────────────────────────────
// Simple IMU integration for motion prediction between scans

struct ImuMeasurement {
    double time;
    V3D acc;
    V3D gyr;
};

class ImuIntegrator {
public:
    std::deque<ImuMeasurement> buffer;
    std::mutex mtx;
    double gravity;

    // Current integrated state
    V3D velocity = V3D::Zero();
    V3D position = V3D::Zero();
    Q4D orientation = Q4D::Identity();
    bool initialized = false;

    ImuIntegrator(double g = 9.80511) : gravity(g) {}

    void addMeasurement(double time, const V3D& acc, const V3D& gyr) {
        std::lock_guard<std::mutex> lock(mtx);
        buffer.push_back({time, acc, gyr});
        // Keep buffer bounded
        while (buffer.size() > 2000) buffer.pop_front();
    }

    // Integrate IMU from last_time to current_time
    // Returns predicted delta rotation and translation
    bool predict(double last_time, double curr_time,
                 const Q4D& last_orientation,
                 Q4D& pred_orientation, V3D& pred_translation) {
        std::lock_guard<std::mutex> lock(mtx);

        pred_orientation = last_orientation;
        pred_translation = V3D::Zero();

        if (buffer.empty()) return false;

        V3D delta_v = V3D::Zero();
        V3D delta_p = V3D::Zero();
        Q4D delta_q = Q4D::Identity();

        double prev_time = last_time;
        V3D gravity_vec(0, 0, -gravity);

        for (const auto& imu : buffer) {
            if (imu.time <= last_time) continue;
            if (imu.time > curr_time) break;

            double dt = imu.time - prev_time;
            if (dt <= 0 || dt > 0.5) {
                prev_time = imu.time;
                continue;
            }

            // Integrate gyroscope (rotation)
            V3D half_angle = imu.gyr * dt * 0.5;
            double angle = half_angle.norm();
            Q4D dq;
            if (angle > 1e-10) {
                dq = Q4D(Eigen::AngleAxisd(imu.gyr.norm() * dt, imu.gyr.normalized()));
            } else {
                dq = Q4D::Identity();
            }
            delta_q = delta_q * dq;
            delta_q.normalize();

            // Integrate accelerometer (velocity and position)
            V3D acc_world = (last_orientation * delta_q) * imu.acc + gravity_vec;
            // Use velocity BEFORE update for position (midpoint integration)
            delta_p += delta_v * dt + 0.5 * acc_world * dt * dt;
            delta_v += acc_world * dt;

            prev_time = imu.time;
        }

        pred_orientation = last_orientation * delta_q;
        pred_orientation.normalize();
        pred_translation = delta_p;
        return true;
    }
};

// ─── SLAM Core ───────────────────────────────────────────────────────────────

class AriseSLAM {
public:
    SLAMConfig config;
    LocalMap local_map;
    ImuIntegrator imu_integrator;

    // Current state
    V3D position = V3D::Zero();
    Q4D orientation = Q4D::Identity();
    double last_scan_time = -1.0;
    bool initialized = false;
    int frame_count = 0;

    AriseSLAM(const SLAMConfig& cfg)
        : config(cfg),
          local_map(cfg.max_range, cfg.line_res, cfg.plane_res),
          imu_integrator(cfg.gravity) {
        // Set initial pose
        position = V3D(cfg.init_x, cfg.init_y, cfg.init_z);
        orientation = Q4D(
            Eigen::AngleAxisd(cfg.init_yaw * DEG2RAD, V3D::UnitZ()) *
            Eigen::AngleAxisd(cfg.init_pitch * DEG2RAD, V3D::UnitY()) *
            Eigen::AngleAxisd(cfg.init_roll * DEG2RAD, V3D::UnitX())
        );
    }

    // Process a new point cloud scan
    // Returns true if pose was updated
    bool processScan(const CloudType::Ptr& raw_cloud, double timestamp) {
        if (raw_cloud->empty()) return false;

        // Filter: remove NaN, near, far points
        CloudType::Ptr filtered(new CloudType);
        filtered->reserve(raw_cloud->size());
        double blind_sq = config.blind_distance * config.blind_distance;
        double max_sq = config.max_range * config.max_range;
        for (const auto& pt : *raw_cloud) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;
            double r_sq = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
            if (r_sq < blind_sq || r_sq > max_sq) continue;
            filtered->push_back(pt);
        }

        if (filtered->size() < 100) {
            printf("[SLAM] Too few points after filtering: %zu\n", filtered->size());
            return false;
        }

        // Downsample input cloud
        if (config.scan_voxel_size > 0) {
            pcl::VoxelGrid<PointType> vg;
            vg.setLeafSize(config.scan_voxel_size, config.scan_voxel_size,
                           config.scan_voxel_size);
            vg.setInputCloud(filtered);
            vg.filter(*filtered);
        }

        // Extract features
        FeatureSet features = extractFeatures(filtered, config);

        if (static_cast<int>(features.edges->size()) < config.edge_feature_min &&
            static_cast<int>(features.planes->size()) < config.surf_feature_min) {
            printf("[SLAM] Insufficient features: edges=%zu planes=%zu\n",
                   features.edges->size(), features.planes->size());
            // Still use full cloud for first frame
            if (initialized) return false;
        }

        if (!initialized) {
            // First frame: just initialize the map
            CloudType::Ptr world_edges(new CloudType);
            CloudType::Ptr world_planes(new CloudType);
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.linear() = orientation.toRotationMatrix();
            T.translation() = position;

            pcl::transformPointCloud(*features.edges, *world_edges, T);
            pcl::transformPointCloud(*features.planes, *world_planes, T);

            local_map.addEdgeCloud(world_edges, position);
            local_map.addSurfCloud(world_planes, position);

            initialized = true;
            last_scan_time = timestamp;
            frame_count++;
            printf("[SLAM] Initialized at (%.1f, %.1f, %.1f) with %zu edge + %zu plane features\n",
                   position.x(), position.y(), position.z(),
                   features.edges->size(), features.planes->size());
            return true;
        }

        // IMU prediction for initial guess
        Q4D pred_orientation = orientation;
        V3D pred_translation = V3D::Zero();
        if (config.use_imu && last_scan_time > 0) {
            imu_integrator.predict(last_scan_time, timestamp,
                                   orientation, pred_orientation, pred_translation);
        }

        V3D pred_position = position + pred_translation;

        // Scan-to-map matching via Ceres optimization
        bool match_success = matchScanToMap(features, pred_position, pred_orientation);

        if (!match_success) {
            // Use prediction as fallback
            position = pred_position;
            orientation = pred_orientation;
            printf("[SLAM] Frame %d: matching failed, using prediction\n", frame_count);
        }

        // Update map with new features
        CloudType::Ptr world_edges(new CloudType);
        CloudType::Ptr world_planes(new CloudType);
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.linear() = orientation.toRotationMatrix();
        T.translation() = position;

        pcl::transformPointCloud(*features.edges, *world_edges, T);
        pcl::transformPointCloud(*features.planes, *world_planes, T);

        local_map.addEdgeCloud(world_edges, position);
        local_map.addSurfCloud(world_planes, position);

        last_scan_time = timestamp;
        frame_count++;
        return true;
    }

private:
    // Core scan-to-map matching
    bool matchScanToMap(const FeatureSet& features,
                        V3D& position_inout, Q4D& orientation_inout) {
        if (!local_map.edge_tree_valid && !local_map.surf_tree_valid) {
            return false;
        }

        // Pose parameters: [tx, ty, tz, qx, qy, qz, qw]
        double params[7];
        params[0] = position_inout.x();
        params[1] = position_inout.y();
        params[2] = position_inout.z();
        params[3] = orientation_inout.x();
        params[4] = orientation_inout.y();
        params[5] = orientation_inout.z();
        params[6] = orientation_inout.w();

        // ICP outer loop
        for (int iter = 0; iter < config.max_icp_iterations; ++iter) {
            ceres::Problem problem;
            problem.AddParameterBlock(params, 7, new PoseSE3Manifold());

            Q4D q_curr(params[6], params[3], params[4], params[5]);
            q_curr.normalize();
            V3D t_curr(params[0], params[1], params[2]);

            int edge_count = 0;
            int surf_count = 0;

            // Edge feature matching
            if (local_map.edge_tree_valid && features.edges->size() > 0) {
                for (const auto& pt : *features.edges) {
                    // Transform point to world frame using current estimate
                    V3D p_body(pt.x, pt.y, pt.z);
                    V3D p_world = q_curr * p_body + t_curr;

                    PointType search_pt;
                    search_pt.x = p_world.x();
                    search_pt.y = p_world.y();
                    search_pt.z = p_world.z();

                    std::vector<int> nn_indices;
                    std::vector<float> nn_dists;
                    local_map.edge_kdtree.nearestKSearch(
                        search_pt, config.edge_nbr_neighbors, nn_indices, nn_dists);

                    if (nn_indices.size() < 2) continue;
                    if (nn_dists.back() > config.max_edge_distance * config.max_edge_distance)
                        continue;

                    // Fit line using PCA on nearest neighbors
                    V3D center = V3D::Zero();
                    for (int idx : nn_indices) {
                        const auto& mp = local_map.edge_map->points[idx];
                        center += V3D(mp.x, mp.y, mp.z);
                    }
                    center /= nn_indices.size();

                    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                    for (int idx : nn_indices) {
                        const auto& mp = local_map.edge_map->points[idx];
                        V3D d = V3D(mp.x, mp.y, mp.z) - center;
                        cov += d * d.transpose();
                    }
                    cov /= nn_indices.size();

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
                    V3D eigenvalues = es.eigenvalues();

                    // Check line-ness: largest eigenvalue >> others
                    if (eigenvalues(2) < 3.0 * eigenvalues(1)) continue;

                    // Line direction = eigenvector of largest eigenvalue
                    V3D line_dir = es.eigenvectors().col(2).normalized();

                    // Two points on the line
                    V3D lp_a = center + 0.1 * line_dir;
                    V3D lp_b = center - 0.1 * line_dir;

                    problem.AddResidualBlock(
                        new EdgeCostFunction(p_body, lp_a, lp_b),
                        new ceres::HuberLoss(0.1),
                        params);
                    edge_count++;
                }
            }

            // Surface feature matching
            if (local_map.surf_tree_valid && features.planes->size() > 0) {
                for (const auto& pt : *features.planes) {
                    V3D p_body(pt.x, pt.y, pt.z);
                    V3D p_world = q_curr * p_body + t_curr;

                    PointType search_pt;
                    search_pt.x = p_world.x();
                    search_pt.y = p_world.y();
                    search_pt.z = p_world.z();

                    std::vector<int> nn_indices;
                    std::vector<float> nn_dists;
                    local_map.surf_kdtree.nearestKSearch(
                        search_pt, config.surf_nbr_neighbors, nn_indices, nn_dists);

                    if (nn_indices.size() < 3) continue;
                    if (nn_dists.back() > config.max_surf_distance * config.max_surf_distance)
                        continue;

                    // Fit plane using PCA on nearest neighbors
                    V3D center = V3D::Zero();
                    for (int idx : nn_indices) {
                        const auto& mp = local_map.surf_map->points[idx];
                        center += V3D(mp.x, mp.y, mp.z);
                    }
                    center /= nn_indices.size();

                    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                    for (int idx : nn_indices) {
                        const auto& mp = local_map.surf_map->points[idx];
                        V3D d = V3D(mp.x, mp.y, mp.z) - center;
                        cov += d * d.transpose();
                    }
                    cov /= nn_indices.size();

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
                    V3D eigenvalues = es.eigenvalues();

                    // Check plane-ness: smallest eigenvalue << others
                    if (eigenvalues(0) > 0.01 * eigenvalues(1)) continue;

                    // Plane normal = eigenvector of smallest eigenvalue
                    V3D normal = es.eigenvectors().col(0).normalized();
                    double d = normal.dot(center);

                    problem.AddResidualBlock(
                        new SurfCostFunction(p_body, normal, d),
                        new ceres::HuberLoss(0.1),
                        params);
                    surf_count++;
                }
            }

            if (edge_count + surf_count < 10) {
                printf("[SLAM] Too few correspondences: edges=%d planes=%d\n",
                       edge_count, surf_count);
                return false;
            }

            // Solve
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = config.max_lm_iterations;
            options.minimizer_progress_to_stdout = false;
            options.num_threads = 2;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            if (summary.termination_type == ceres::CONVERGENCE ||
                summary.termination_type == ceres::NO_CONVERGENCE) {
                // Normalize quaternion after optimization
                double qnorm = std::sqrt(params[3]*params[3] + params[4]*params[4] +
                                         params[5]*params[5] + params[6]*params[6]);
                if (qnorm > 1e-10) {
                    params[3] /= qnorm;
                    params[4] /= qnorm;
                    params[5] /= qnorm;
                    params[6] /= qnorm;
                }
            }
        }

        // Update output pose
        position_inout = V3D(params[0], params[1], params[2]);
        orientation_inout = Q4D(params[6], params[3], params[4], params[5]);
        orientation_inout.normalize();

        // Update class state
        position = position_inout;
        orientation = orientation_inout;

        return true;
    }
};

// ─── LCM Handler ─────────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};
void signal_handler(int) { g_running = false; }

struct SLAMHandler {
    lcm::LCM* lcm;
    AriseSLAM* slam;
    std::string topic_registered_scan;
    std::string topic_odometry;
    std::string topic_map;
    SLAMConfig config;

    std::mutex mtx;
    double last_publish_time = 0.0;
    double last_map_publish_time = 0.0;

    void onRawPoints(const lcm::ReceiveBuffer*, const std::string&,
                     const sensor_msgs::PointCloud2* msg) {
        std::lock_guard<std::mutex> lock(mtx);

        double scan_time = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;

        // Convert to PCL
        CloudType::Ptr cloud(new CloudType);
        smartnav::to_pcl(*msg, *cloud);

        if (cloud->empty()) return;

        // Process scan
        bool updated = slam->processScan(cloud, scan_time);

        if (!updated) return;

        // Rate-limit publishing
        if (scan_time - last_publish_time < config.min_publish_interval) return;
        last_publish_time = scan_time;

        // Publish odometry
        publishOdometry(scan_time);

        // Publish registered scan (transform raw cloud to world frame)
        publishRegisteredScan(*msg, scan_time);

        // Publish map periodically
        if (config.publish_map && config.map_publish_rate > 0) {
            double now = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            double interval = 1.0 / config.map_publish_rate;
            if (now - last_map_publish_time > interval) {
                publishMap(scan_time);
                last_map_publish_time = now;
            }
        }
    }

    void onImu(const lcm::ReceiveBuffer*, const std::string&,
               const sensor_msgs::Imu* msg) {
        double imu_time = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;
        V3D acc(msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z);
        V3D gyr(msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
        slam->imu_integrator.addMeasurement(imu_time, acc, gyr);
    }

    void publishOdometry(double timestamp) {
        Q4D q = slam->orientation;
        V3D t = slam->position;

        nav_msgs::Odometry odom;
        odom.header = dimos::make_header("map", timestamp);
        odom.child_frame_id = "sensor";
        odom.pose.pose.position.x = t.x();
        odom.pose.pose.position.y = t.y();
        odom.pose.pose.position.z = t.z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        lcm->publish(topic_odometry, &odom);
    }

    void publishRegisteredScan(const sensor_msgs::PointCloud2& raw_msg,
                               double timestamp) {
        // Transform raw cloud to world frame
        CloudType::Ptr raw_cloud(new CloudType);
        smartnav::to_pcl(raw_msg, *raw_cloud);

        if (raw_cloud->empty()) return;

        // Downsample for output
        if (config.scan_voxel_size > 0) {
            pcl::VoxelGrid<PointType> vg;
            vg.setLeafSize(config.scan_voxel_size, config.scan_voxel_size,
                           config.scan_voxel_size);
            vg.setInputCloud(raw_cloud);
            vg.filter(*raw_cloud);
        }

        CloudType::Ptr world_cloud(new CloudType);
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.linear() = slam->orientation.toRotationMatrix();
        T.translation() = slam->position;
        pcl::transformPointCloud(*raw_cloud, *world_cloud, T);

        sensor_msgs::PointCloud2 out_msg = smartnav::from_pcl(*world_cloud, "map", timestamp);
        lcm->publish(topic_registered_scan, &out_msg);
    }

    void publishMap(double timestamp) {
        CloudType::Ptr map_cloud = slam->local_map.getMapCloud(config.map_viz_voxel_size);
        if (map_cloud->empty()) return;

        sensor_msgs::PointCloud2 out_msg = smartnav::from_pcl(*map_cloud, "map", timestamp);
        lcm->publish(topic_map, &out_msg);

        printf("[SLAM] Map published: %zu points (edges=%zu surfs=%zu)\n",
               map_cloud->size(), slam->local_map.edge_map->size(),
               slam->local_map.surf_map->size());
    }
};

// ─── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    dimos::NativeModule mod(argc, argv);

    // Read config from CLI args
    SLAMConfig config;
    config.edge_threshold       = mod.arg_float("edgeThreshold", 1.0f);
    config.surf_threshold       = mod.arg_float("surfThreshold", 0.1f);
    config.edge_feature_min     = mod.arg_int("edgeFeatureMinValidNum", 10);
    config.surf_feature_min     = mod.arg_int("surfFeatureMinValidNum", 100);
    config.scan_voxel_size      = mod.arg_float("scanVoxelSize", 0.1f);
    config.line_res             = mod.arg_float("lineRes", 0.2f);
    config.plane_res            = mod.arg_float("planeRes", 0.4f);
    config.max_icp_iterations   = mod.arg_int("maxIcpIterations", 4);
    config.max_lm_iterations    = mod.arg_int("maxLmIterations", 15);
    config.edge_nbr_neighbors   = mod.arg_int("edgeNbrNeighbors", 5);
    config.surf_nbr_neighbors   = mod.arg_int("surfNbrNeighbors", 5);
    config.max_edge_distance    = mod.arg_float("maxEdgeDistance", 1.0f);
    config.max_surf_distance    = mod.arg_float("maxSurfDistance", 1.0f);
    config.use_imu              = mod.arg_bool("useImu", true);
    config.gravity              = mod.arg_float("gravity", 9.80511f);
    config.min_publish_interval = mod.arg_float("minPublishInterval", 0.05f);
    config.publish_map          = mod.arg_bool("publishMap", false);
    config.map_publish_rate     = mod.arg_float("mapPublishRate", 0.2f);
    config.map_viz_voxel_size   = mod.arg_float("mapVizVoxelSize", 0.2f);
    config.max_range            = mod.arg_float("maxRange", 100.0f);
    config.blind_distance       = mod.arg_float("blindDistance", 0.5f);
    config.init_x               = mod.arg_float("initX", 0.0f);
    config.init_y               = mod.arg_float("initY", 0.0f);
    config.init_z               = mod.arg_float("initZ", 0.0f);
    config.init_roll            = mod.arg_float("initRoll", 0.0f);
    config.init_pitch           = mod.arg_float("initPitch", 0.0f);
    config.init_yaw             = mod.arg_float("initYaw", 0.0f);

    printf("[SLAM] Config: edgeThreshold=%.2f surfThreshold=%.2f "
           "maxIcpIterations=%d scanVoxelSize=%.2f maxRange=%.0f useImu=%s\n",
           config.edge_threshold, config.surf_threshold,
           config.max_icp_iterations, config.scan_voxel_size,
           config.max_range, config.use_imu ? "true" : "false");

    // Create SLAM instance
    AriseSLAM slam(config);

    // LCM setup
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[SLAM] LCM initialization failed\n");
        return 1;
    }

    SLAMHandler handler;
    handler.lcm = &lcm;
    handler.slam = &slam;
    handler.topic_registered_scan = mod.topic("registered_scan");
    handler.topic_odometry = mod.topic("odometry");
    handler.topic_map = mod.has("local_map") ? mod.topic("local_map") : "";
    handler.config = config;

    std::string topic_raw = mod.topic("raw_points");
    lcm.subscribe(topic_raw, &SLAMHandler::onRawPoints, &handler);

    if (mod.has("imu")) {
        std::string topic_imu = mod.topic("imu");
        lcm.subscribe(topic_imu, &SLAMHandler::onImu, &handler);
        printf("[SLAM] IMU subscribed on: %s\n", topic_imu.c_str());
    }

    printf("[SLAM] Listening on: raw_points=%s\n", topic_raw.c_str());
    printf("[SLAM] Publishing:   registered_scan=%s odometry=%s\n",
           handler.topic_registered_scan.c_str(), handler.topic_odometry.c_str());

    while (g_running) {
        lcm.handleTimeout(100);
    }

    printf("[SLAM] Shutting down. Frames processed: %d\n", slam.frame_count);
    return 0;
}
