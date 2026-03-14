// Terrain Analysis — dimos NativeModule port
// Ported from ROS2: src/base_autonomy/terrain_analysis/src/terrainAnalysis.cpp
//
// Classifies terrain into ground vs obstacle using a rolling voxel grid,
// planar elevation estimation, and dynamic-obstacle filtering.
// Publishes the terrain map as a PointCloud2 (intensity = elevation above ground).

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

#include "sensor_msgs/PointCloud2.hpp"
#include "nav_msgs/Odometry.hpp"

#ifdef USE_PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif

using namespace std;

const double PI = 3.1415926;

// --- Configuration parameters (populated from CLI args) ---
double scanVoxelSize = 0.05;
double decayTime = 2.0;
double noDecayDis = 4.0;
double clearingDis = 8.0;
bool clearingCloud = false;
bool useSorting = true;
double quantileZ = 0.25;
bool considerDrop = false;
bool limitGroundLift = false;
double maxGroundLift = 0.15;
bool clearDyObs = false;
double minDyObsDis = 0.3;
double absDyObsRelZThre = 0.2;
double minDyObsVFOV = -16.0;
double maxDyObsVFOV = 16.0;
int minDyObsPointNum = 1;
int minOutOfFovPointNum = 2;
double obstacleHeightThre = 0.2;
bool noDataObstacle = false;
int noDataBlockSkipNum = 0;
int minBlockPointNum = 10;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2;

// --- Terrain voxel parameters ---
float terrainVoxelSize = 1.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

// --- Planar voxel parameters ---
float planarVoxelSize = 0.2;
const int planarVoxelWidth = 51;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;

// --- Point cloud storage ---
#ifdef USE_PCL
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
#else
// Lightweight mode: use std::vector<smartnav::PointXYZI>
std::vector<smartnav::PointXYZI> laserCloud;
std::vector<smartnav::PointXYZI> laserCloudCrop;
std::vector<smartnav::PointXYZI> laserCloudDwz;
std::vector<smartnav::PointXYZI> terrainCloud;
std::vector<smartnav::PointXYZI> terrainCloudElev;
std::vector<smartnav::PointXYZI> terrainVoxelCloudVec[terrainVoxelNum];
#endif

// --- Per-voxel bookkeeping ---
int terrainVoxelUpdateNum[terrainVoxelNum] = {0};
float terrainVoxelUpdateTime[terrainVoxelNum] = {0};
float planarVoxelElev[planarVoxelNum] = {0};
int planarVoxelEdge[planarVoxelNum] = {0};
int planarVoxelDyObs[planarVoxelNum] = {0};
int planarVoxelOutOfFov[planarVoxelNum] = {0};
vector<float> planarPointElev[planarVoxelNum];

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;
int noDataInited = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

// ============================================================
// LCM message handlers
// ============================================================

class TerrainAnalysisHandler {
public:
    // State estimation (odometry) callback
    void odometryHandler(const lcm::ReceiveBuffer* /*rbuf*/,
                         const std::string& /*channel*/,
                         const nav_msgs::Odometry* odom) {
        double roll, pitch, yaw;
        smartnav::quat_to_rpy(
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z,
            odom->pose.pose.orientation.w,
            roll, pitch, yaw);

        vehicleRoll = roll;
        vehiclePitch = pitch;
        vehicleYaw = yaw;
        vehicleX = odom->pose.pose.position.x;
        vehicleY = odom->pose.pose.position.y;
        vehicleZ = odom->pose.pose.position.z;

        sinVehicleRoll = sin(vehicleRoll);
        cosVehicleRoll = cos(vehicleRoll);
        sinVehiclePitch = sin(vehiclePitch);
        cosVehiclePitch = cos(vehiclePitch);
        sinVehicleYaw = sin(vehicleYaw);
        cosVehicleYaw = cos(vehicleYaw);

        if (noDataInited == 0) {
            vehicleXRec = vehicleX;
            vehicleYRec = vehicleY;
            noDataInited = 1;
        }
        if (noDataInited == 1) {
            float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                             (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
            if (dis >= noDecayDis)
                noDataInited = 2;
        }
    }

    // Registered laser scan callback
    void laserCloudHandler(const lcm::ReceiveBuffer* /*rbuf*/,
                           const std::string& /*channel*/,
                           const sensor_msgs::PointCloud2* laserCloud2) {
        laserCloudTime = smartnav::get_timestamp(*laserCloud2);
        if (!systemInited) {
            systemInitTime = laserCloudTime;
            systemInited = true;
        }

#ifdef USE_PCL
        // Convert LCM PointCloud2 to PCL
        smartnav::to_pcl(*laserCloud2, *laserCloud);

        pcl::PointXYZI point;
        laserCloudCrop->clear();
        int laserCloudSize = laserCloud->points.size();
        for (int i = 0; i < laserCloudSize; i++) {
            point = laserCloud->points[i];

            float pointX = point.x;
            float pointY = point.y;
            float pointZ = point.z;

            float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                             (pointY - vehicleY) * (pointY - vehicleY));
            if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
                pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
                dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
                point.x = pointX;
                point.y = pointY;
                point.z = pointZ;
                point.intensity = laserCloudTime - systemInitTime;
                laserCloudCrop->push_back(point);
            }
        }
#else
        // Lightweight mode: parse directly
        auto pts = smartnav::parse_pointcloud2(*laserCloud2);
        laserCloud.assign(pts.begin(), pts.end());

        laserCloudCrop.clear();
        for (size_t i = 0; i < laserCloud.size(); i++) {
            smartnav::PointXYZI point = laserCloud[i];

            float pointX = point.x;
            float pointY = point.y;
            float pointZ = point.z;

            float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                             (pointY - vehicleY) * (pointY - vehicleY));
            if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
                pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
                dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
                point.intensity = laserCloudTime - systemInitTime;
                laserCloudCrop.push_back(point);
            }
        }
#endif

        newlaserCloud = true;
    }
};

// ============================================================
// Non-PCL voxel downsampling helper (used when USE_PCL is off)
// ============================================================
#ifndef USE_PCL
static void downsample_voxel(const std::vector<smartnav::PointXYZI>& input,
                             std::vector<smartnav::PointXYZI>& output,
                             float leafSize) {
    output.clear();
    if (input.empty()) return;

    // Simple hash-based voxel grid filter
    struct VoxelKey {
        int ix, iy, iz;
        bool operator==(const VoxelKey& o) const {
            return ix == o.ix && iy == o.iy && iz == o.iz;
        }
    };
    struct VoxelHash {
        size_t operator()(const VoxelKey& k) const {
            size_t h = 0;
            h ^= std::hash<int>()(k.ix) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<int>()(k.iy) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<int>()(k.iz) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };
    struct Accum {
        double sx, sy, sz, si;
        int count;
    };

    std::unordered_map<VoxelKey, Accum, VoxelHash> grid;
    float inv = 1.0f / leafSize;
    for (const auto& p : input) {
        VoxelKey k;
        k.ix = (int)floor(p.x * inv);
        k.iy = (int)floor(p.y * inv);
        k.iz = (int)floor(p.z * inv);
        auto& a = grid[k];
        a.sx += p.x; a.sy += p.y; a.sz += p.z; a.si += p.intensity;
        a.count++;
    }
    output.reserve(grid.size());
    for (const auto& kv : grid) {
        const auto& a = kv.second;
        float n = (float)a.count;
        output.push_back({(float)(a.sx / n), (float)(a.sy / n),
                          (float)(a.sz / n), (float)(a.si / n)});
    }
}
#endif

// ============================================================
// main
// ============================================================

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // --- Topic names from CLI args ---
    std::string odometry_topic    = mod.topic("odometry");
    std::string registered_scan_topic = mod.topic("registered_scan");
    std::string terrain_map_topic = mod.topic("terrain_map");

    // --- Load configuration parameters ---
    scanVoxelSize     = mod.arg_float("scanVoxelSize",     (float)scanVoxelSize);
    decayTime         = mod.arg_float("decayTime",         (float)decayTime);
    noDecayDis        = mod.arg_float("noDecayDis",        (float)noDecayDis);
    clearingDis       = mod.arg_float("clearingDis",       (float)clearingDis);
    useSorting        = mod.arg_bool("useSorting",         useSorting);
    quantileZ         = mod.arg_float("quantileZ",         (float)quantileZ);
    considerDrop      = mod.arg_bool("considerDrop",       considerDrop);
    limitGroundLift   = mod.arg_bool("limitGroundLift",    limitGroundLift);
    maxGroundLift     = mod.arg_float("maxGroundLift",     (float)maxGroundLift);
    clearDyObs        = mod.arg_bool("clearDyObs",         clearDyObs);
    minDyObsDis       = mod.arg_float("minDyObsDis",       (float)minDyObsDis);
    absDyObsRelZThre  = mod.arg_float("absDyObsRelZThre", (float)absDyObsRelZThre);
    minDyObsVFOV      = mod.arg_float("minDyObsVFOV",     (float)minDyObsVFOV);
    maxDyObsVFOV      = mod.arg_float("maxDyObsVFOV",     (float)maxDyObsVFOV);
    minDyObsPointNum  = mod.arg_int("minDyObsPointNum",   minDyObsPointNum);
    minOutOfFovPointNum = mod.arg_int("minOutOfFovPointNum", minOutOfFovPointNum);
    obstacleHeightThre = mod.arg_float("obstacleHeightThre", (float)obstacleHeightThre);
    noDataObstacle    = mod.arg_bool("noDataObstacle",     noDataObstacle);
    noDataBlockSkipNum = mod.arg_int("noDataBlockSkipNum", noDataBlockSkipNum);
    minBlockPointNum  = mod.arg_int("minBlockPointNum",    minBlockPointNum);
    vehicleHeight     = mod.arg_float("vehicleHeight",     (float)vehicleHeight);
    voxelPointUpdateThre = mod.arg_int("voxelPointUpdateThre", voxelPointUpdateThre);
    voxelTimeUpdateThre  = mod.arg_float("voxelTimeUpdateThre", (float)voxelTimeUpdateThre);
    minRelZ           = mod.arg_float("minRelZ",           (float)minRelZ);
    maxRelZ           = mod.arg_float("maxRelZ",           (float)maxRelZ);
    disRatioZ         = mod.arg_float("disRatioZ",         (float)disRatioZ);

    // --- LCM setup ---
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[terrain_analysis] LCM initialization failed\n");
        return 1;
    }

    TerrainAnalysisHandler handler;
    lcm.subscribe(odometry_topic, &TerrainAnalysisHandler::odometryHandler, &handler);
    lcm.subscribe(registered_scan_topic, &TerrainAnalysisHandler::laserCloudHandler, &handler);

    // --- Initialize terrain voxel clouds ---
#ifdef USE_PCL
    for (int i = 0; i < terrainVoxelNum; i++) {
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);
#else
    for (int i = 0; i < terrainVoxelNum; i++) {
        terrainVoxelCloudVec[i].clear();
    }
#endif

    printf("[terrain_analysis] Started. Listening on '%s' and '%s', publishing to '%s'\n",
           odometry_topic.c_str(), registered_scan_topic.c_str(), terrain_map_topic.c_str());

    // --- Main loop at ~100 Hz ---
    bool running = true;
    while (running) {
        // Handle all pending LCM messages (non-blocking, 10ms timeout)
        lcm.handleTimeout(10);

        if (newlaserCloud) {
            newlaserCloud = false;

            // ========================================================
            // Terrain voxel roll-over to keep grid centered on vehicle
            // ========================================================
            float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

#ifdef USE_PCL
            // Roll over -X direction
            while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
                for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                        terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
                    for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                            terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
                    }
                    terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[indY]->clear();
                }
                terrainVoxelShiftX--;
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            // Roll over +X direction
            while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
                for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                        terrainVoxelCloud[indY];
                    for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                            terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
                    }
                    terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] =
                        terrainVoxelCloudPtr;
                    terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]->clear();
                }
                terrainVoxelShiftX++;
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            // Roll over -Y direction
            while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
                for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                        terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
                    for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                            terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
                    }
                    terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
                }
                terrainVoxelShiftY--;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            // Roll over +Y direction
            while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
                for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                        terrainVoxelCloud[terrainVoxelWidth * indX];
                    for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                            terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
                    }
                    terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] =
                        terrainVoxelCloudPtr;
                    terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]->clear();
                }
                terrainVoxelShiftY++;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            // ========================================================
            // Stack registered laser scans into terrain voxels
            // ========================================================
            pcl::PointXYZI point;
            int laserCloudCropSize = laserCloudCrop->points.size();
            for (int i = 0; i < laserCloudCropSize; i++) {
                point = laserCloudCrop->points[i];

                int indX = int((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) +
                           terrainVoxelHalfWidth;
                int indY = int((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) +
                           terrainVoxelHalfWidth;

                if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
                    indX--;
                if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
                    indY--;

                if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
                    indY < terrainVoxelWidth) {
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
                    terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
                }
            }

            // ========================================================
            // Downsample and decay terrain voxels
            // ========================================================
            for (int ind = 0; ind < terrainVoxelNum; ind++) {
                if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
                    laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                        voxelTimeUpdateThre ||
                    clearingCloud) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                        terrainVoxelCloud[ind];

                    laserCloudDwz->clear();
                    downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
                    downSizeFilter.filter(*laserCloudDwz);

                    terrainVoxelCloudPtr->clear();
                    int laserCloudDwzSize = laserCloudDwz->points.size();
                    for (int i = 0; i < laserCloudDwzSize; i++) {
                        point = laserCloudDwz->points[i];
                        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                                         (point.y - vehicleY) * (point.y - vehicleY));
                        if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                            point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
                            (laserCloudTime - systemInitTime - point.intensity <
                                 decayTime ||
                             dis < noDecayDis) &&
                            !(dis < clearingDis && clearingCloud)) {
                            terrainVoxelCloudPtr->push_back(point);
                        }
                    }

                    terrainVoxelUpdateNum[ind] = 0;
                    terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
                }
            }

            // ========================================================
            // Gather terrain cloud from center 11x11 voxels
            // ========================================================
            terrainCloud->clear();
            for (int indX = terrainVoxelHalfWidth - 5;
                 indX <= terrainVoxelHalfWidth + 5; indX++) {
                for (int indY = terrainVoxelHalfWidth - 5;
                     indY <= terrainVoxelHalfWidth + 5; indY++) {
                    *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
                }
            }

#else  // !USE_PCL — lightweight mode

            // Roll over -X direction
            while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
                for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                    auto tmp = std::move(
                        terrainVoxelCloudVec[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]);
                    for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
                        terrainVoxelCloudVec[terrainVoxelWidth * indX + indY] =
                            std::move(terrainVoxelCloudVec[terrainVoxelWidth * (indX - 1) + indY]);
                    }
                    tmp.clear();
                    terrainVoxelCloudVec[indY] = std::move(tmp);
                }
                terrainVoxelShiftX--;
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            // Roll over +X direction
            while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
                for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                    auto tmp = std::move(terrainVoxelCloudVec[indY]);
                    for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
                        terrainVoxelCloudVec[terrainVoxelWidth * indX + indY] =
                            std::move(terrainVoxelCloudVec[terrainVoxelWidth * (indX + 1) + indY]);
                    }
                    tmp.clear();
                    terrainVoxelCloudVec[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] =
                        std::move(tmp);
                }
                terrainVoxelShiftX++;
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            // Roll over -Y direction
            while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
                for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                    auto tmp = std::move(
                        terrainVoxelCloudVec[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]);
                    for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
                        terrainVoxelCloudVec[terrainVoxelWidth * indX + indY] =
                            std::move(terrainVoxelCloudVec[terrainVoxelWidth * indX + (indY - 1)]);
                    }
                    tmp.clear();
                    terrainVoxelCloudVec[terrainVoxelWidth * indX] = std::move(tmp);
                }
                terrainVoxelShiftY--;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            // Roll over +Y direction
            while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
                for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                    auto tmp = std::move(terrainVoxelCloudVec[terrainVoxelWidth * indX]);
                    for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
                        terrainVoxelCloudVec[terrainVoxelWidth * indX + indY] =
                            std::move(terrainVoxelCloudVec[terrainVoxelWidth * indX + (indY + 1)]);
                    }
                    tmp.clear();
                    terrainVoxelCloudVec[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] =
                        std::move(tmp);
                }
                terrainVoxelShiftY++;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            // ========================================================
            // Stack registered laser scans into terrain voxels
            // ========================================================
            int laserCloudCropSize = (int)laserCloudCrop.size();
            for (int i = 0; i < laserCloudCropSize; i++) {
                smartnav::PointXYZI point = laserCloudCrop[i];

                int indX = int((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) +
                           terrainVoxelHalfWidth;
                int indY = int((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) +
                           terrainVoxelHalfWidth;

                if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
                    indX--;
                if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
                    indY--;

                if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
                    indY < terrainVoxelWidth) {
                    terrainVoxelCloudVec[terrainVoxelWidth * indX + indY].push_back(point);
                    terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
                }
            }

            // ========================================================
            // Downsample and decay terrain voxels
            // ========================================================
            for (int ind = 0; ind < terrainVoxelNum; ind++) {
                if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
                    laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                        voxelTimeUpdateThre ||
                    clearingCloud) {
                    auto& terrainVoxelCloudRef = terrainVoxelCloudVec[ind];

                    downsample_voxel(terrainVoxelCloudRef, laserCloudDwz, scanVoxelSize);

                    terrainVoxelCloudRef.clear();
                    int laserCloudDwzSize = (int)laserCloudDwz.size();
                    for (int i = 0; i < laserCloudDwzSize; i++) {
                        smartnav::PointXYZI point = laserCloudDwz[i];
                        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                                         (point.y - vehicleY) * (point.y - vehicleY));
                        if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                            point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
                            (laserCloudTime - systemInitTime - point.intensity <
                                 decayTime ||
                             dis < noDecayDis) &&
                            !(dis < clearingDis && clearingCloud)) {
                            terrainVoxelCloudRef.push_back(point);
                        }
                    }

                    terrainVoxelUpdateNum[ind] = 0;
                    terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
                }
            }

            // ========================================================
            // Gather terrain cloud from center 11x11 voxels
            // ========================================================
            terrainCloud.clear();
            for (int indX = terrainVoxelHalfWidth - 5;
                 indX <= terrainVoxelHalfWidth + 5; indX++) {
                for (int indY = terrainVoxelHalfWidth - 5;
                     indY <= terrainVoxelHalfWidth + 5; indY++) {
                    auto& vc = terrainVoxelCloudVec[terrainVoxelWidth * indX + indY];
                    terrainCloud.insert(terrainCloud.end(), vc.begin(), vc.end());
                }
            }
#endif  // USE_PCL

            // ========================================================
            // Estimate ground elevation per planar voxel
            // ========================================================
            for (int i = 0; i < planarVoxelNum; i++) {
                planarVoxelElev[i] = 0;
                planarVoxelEdge[i] = 0;
                planarVoxelDyObs[i] = 0;
                planarVoxelOutOfFov[i] = 0;
                planarPointElev[i].clear();
            }

#ifdef USE_PCL
            int terrainCloudSize = terrainCloud->points.size();
            for (int i = 0; i < terrainCloudSize; i++) {
                pcl::PointXYZI point = terrainCloud->points[i];
#else
            int terrainCloudSize = (int)terrainCloud.size();
            for (int i = 0; i < terrainCloudSize; i++) {
                smartnav::PointXYZI point = terrainCloud[i];
#endif
                int indX =
                    int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                    planarVoxelHalfWidth;
                int indY =
                    int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                    planarVoxelHalfWidth;

                if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                    indX--;
                if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                    indY--;

                if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
                    for (int dX = -1; dX <= 1; dX++) {
                        for (int dY = -1; dY <= 1; dY++) {
                            if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                                indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                                    .push_back(point.z);
                            }
                        }
                    }
                }
            }

            // Compute per-voxel ground elevation
            if (useSorting) {
                for (int i = 0; i < planarVoxelNum; i++) {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize > 0) {
                        sort(planarPointElev[i].begin(), planarPointElev[i].end());

                        int quantileID = int(quantileZ * planarPointElevSize);
                        if (quantileID < 0)
                            quantileID = 0;
                        else if (quantileID >= planarPointElevSize)
                            quantileID = planarPointElevSize - 1;

                        if (planarPointElev[i][quantileID] >
                                planarPointElev[i][0] + maxGroundLift &&
                            limitGroundLift) {
                            planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
                        } else {
                            planarVoxelElev[i] = planarPointElev[i][quantileID];
                        }
                    }
                }
            } else {
                for (int i = 0; i < planarVoxelNum; i++) {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize > 0) {
                        float minZ = 1000.0;
                        int minID = -1;
                        for (int j = 0; j < planarPointElevSize; j++) {
                            if (planarPointElev[i][j] < minZ) {
                                minZ = planarPointElev[i][j];
                                minID = j;
                            }
                        }

                        if (minID != -1) {
                            planarVoxelElev[i] = planarPointElev[i][minID];
                        }
                    }
                }
            }

            // ========================================================
            // Dynamic obstacle clearing
            // ========================================================
            if (clearDyObs) {
                for (int i = 0; i < terrainCloudSize; i++) {
#ifdef USE_PCL
                    pcl::PointXYZI point = terrainCloud->points[i];
#else
                    smartnav::PointXYZI point = terrainCloud[i];
#endif

                    int indX =
                        int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                        planarVoxelHalfWidth;
                    int indY =
                        int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                        planarVoxelHalfWidth;

                    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                        indX--;
                    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                        indY--;

                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                        indY < planarVoxelWidth) {
                        float pointX1 = point.x - vehicleX;
                        float pointY1 = point.y - vehicleY;
                        float pointZ1 = point.z - vehicleZ;

                        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
                        if (dis1 > minDyObsDis) {
                            float h1 = point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
                            if (h1 > obstacleHeightThre) {
                                float pointX2 =
                                    pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                                float pointY2 =
                                    -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                                float pointZ2 = pointZ1;

                                float pointX3 =
                                    pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                                float pointY3 = pointY2;
                                float pointZ3 =
                                    pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                                float pointX4 = pointX3;
                                float pointY4 =
                                    pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                                float pointZ4 =
                                    -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                                float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;
                                if ((angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV) || fabs(pointZ4) < absDyObsRelZThre) {
                                    planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
                                } else if (angle4 <= minDyObsVFOV) {
                                    planarVoxelOutOfFov[planarVoxelWidth * indX + indY]++;
                                }
                            }
                        } else {
                            planarVoxelDyObs[planarVoxelWidth * indX + indY] += minDyObsPointNum;
                        }
                    }
                }

                // Mark current-frame high points as dynamic
#ifdef USE_PCL
                int laserCloudCropSz = laserCloudCrop->points.size();
                for (int i = 0; i < laserCloudCropSz; i++) {
                    pcl::PointXYZI point = laserCloudCrop->points[i];
#else
                int laserCloudCropSz = (int)laserCloudCrop.size();
                for (int i = 0; i < laserCloudCropSz; i++) {
                    smartnav::PointXYZI point = laserCloudCrop[i];
#endif
                    int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                               planarVoxelHalfWidth;
                    int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                               planarVoxelHalfWidth;

                    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                        indX--;
                    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                        indY--;

                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                        indY < planarVoxelWidth) {
                        float h1 = point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
                        if (h1 > obstacleHeightThre) {
                            planarVoxelDyObs[planarVoxelWidth * indX + indY] = -1;
                        }
                    }
                }
            }

            // ========================================================
            // Build output: terrain cloud with elevation as intensity
            // ========================================================
#ifdef USE_PCL
            terrainCloudElev->clear();
            int terrainCloudElevSize = 0;
            for (int i = 0; i < terrainCloudSize; i++) {
                pcl::PointXYZI point = terrainCloud->points[i];
                if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
                    int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                               planarVoxelHalfWidth;
                    int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                               planarVoxelHalfWidth;

                    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                        indX--;
                    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                        indY--;

                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                        indY < planarVoxelWidth) {
                        int dyObsPointNum = planarVoxelDyObs[planarVoxelWidth * indX + indY];
                        if (dyObsPointNum < minDyObsPointNum || !clearDyObs) {
                            float disZ =
                                point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
                            if (considerDrop)
                                disZ = fabs(disZ);
                            int planarPointElevSize =
                                planarPointElev[planarVoxelWidth * indX + indY].size();
                            int outOfFovPointNum = planarVoxelOutOfFov[planarVoxelWidth * indX + indY];
                            if (disZ >= 0 && disZ < vehicleHeight && planarPointElevSize >= minBlockPointNum &&
                                (outOfFovPointNum >= minOutOfFovPointNum || disZ < obstacleHeightThre || dyObsPointNum < 0 || !clearDyObs)) {
                                terrainCloudElev->push_back(point);
                                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;
                                terrainCloudElevSize++;
                            }
                        }
                    }
                }
            }
#else
            terrainCloudElev.clear();
            for (int i = 0; i < terrainCloudSize; i++) {
                smartnav::PointXYZI point = terrainCloud[i];
                if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
                    int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                               planarVoxelHalfWidth;
                    int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                               planarVoxelHalfWidth;

                    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                        indX--;
                    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                        indY--;

                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                        indY < planarVoxelWidth) {
                        int dyObsPointNum = planarVoxelDyObs[planarVoxelWidth * indX + indY];
                        if (dyObsPointNum < minDyObsPointNum || !clearDyObs) {
                            float disZ =
                                point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
                            if (considerDrop)
                                disZ = fabs(disZ);
                            int planarPointElevSize =
                                planarPointElev[planarVoxelWidth * indX + indY].size();
                            int outOfFovPointNum = planarVoxelOutOfFov[planarVoxelWidth * indX + indY];
                            if (disZ >= 0 && disZ < vehicleHeight && planarPointElevSize >= minBlockPointNum &&
                                (outOfFovPointNum >= minOutOfFovPointNum || disZ < obstacleHeightThre || dyObsPointNum < 0 || !clearDyObs)) {
                                point.intensity = disZ;
                                terrainCloudElev.push_back(point);
                            }
                        }
                    }
                }
            }
#endif

            // ========================================================
            // No-data obstacle fill
            // ========================================================
            if (noDataObstacle && noDataInited == 2) {
                for (int i = 0; i < planarVoxelNum; i++) {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize < minBlockPointNum) {
                        planarVoxelEdge[i] = 1;
                    }
                }

                for (int noDataBlockSkipCount = 0;
                     noDataBlockSkipCount < noDataBlockSkipNum;
                     noDataBlockSkipCount++) {
                    for (int i = 0; i < planarVoxelNum; i++) {
                        if (planarVoxelEdge[i] >= 1) {
                            int indX = int(i / planarVoxelWidth);
                            int indY = i % planarVoxelWidth;
                            bool edgeVoxel = false;
                            for (int dX = -1; dX <= 1; dX++) {
                                for (int dY = -1; dY <= 1; dY++) {
                                    if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                                        indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                                        if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                                            dY] < planarVoxelEdge[i]) {
                                            edgeVoxel = true;
                                        }
                                    }
                                }
                            }

                            if (!edgeVoxel)
                                planarVoxelEdge[i]++;
                        }
                    }
                }

                for (int i = 0; i < planarVoxelNum; i++) {
                    if (planarVoxelEdge[i] > noDataBlockSkipNum) {
                        int indX = int(i / planarVoxelWidth);
                        int indY = i % planarVoxelWidth;

#ifdef USE_PCL
                        pcl::PointXYZI point;
#else
                        smartnav::PointXYZI point;
#endif
                        point.x =
                            planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
                        point.y =
                            planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
                        point.z = vehicleZ;
                        point.intensity = vehicleHeight;

                        point.x -= planarVoxelSize / 4.0;
                        point.y -= planarVoxelSize / 4.0;
#ifdef USE_PCL
                        terrainCloudElev->push_back(point);
#else
                        terrainCloudElev.push_back(point);
#endif

                        point.x += planarVoxelSize / 2.0;
#ifdef USE_PCL
                        terrainCloudElev->push_back(point);
#else
                        terrainCloudElev.push_back(point);
#endif

                        point.y += planarVoxelSize / 2.0;
#ifdef USE_PCL
                        terrainCloudElev->push_back(point);
#else
                        terrainCloudElev.push_back(point);
#endif

                        point.x -= planarVoxelSize / 2.0;
#ifdef USE_PCL
                        terrainCloudElev->push_back(point);
#else
                        terrainCloudElev.push_back(point);
#endif
                    }
                }
            }

            clearingCloud = false;

            // ========================================================
            // Publish terrain map as PointCloud2 via LCM
            // ========================================================
#ifdef USE_PCL
            sensor_msgs::PointCloud2 terrainCloud2 =
                smartnav::from_pcl(*terrainCloudElev, "map", laserCloudTime);
#else
            sensor_msgs::PointCloud2 terrainCloud2 =
                smartnav::build_pointcloud2(terrainCloudElev, "map", laserCloudTime);
#endif
            lcm.publish(terrain_map_topic, &terrainCloud2);
        }

        // Sleep briefly to yield CPU when no data is ready (~100 Hz loop)
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    return 0;
}
