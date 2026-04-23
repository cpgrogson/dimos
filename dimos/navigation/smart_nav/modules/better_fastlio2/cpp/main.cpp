// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// BetterFastLio2 native module for dimos NativeModule framework.
//
// Faithful reimplementation of https://github.com/Yixin-F/better_fastlio2
// adapted to DimOS: ROS replaced by LCM pub/sub, sensor SDK replaced by
// LCM input subscriptions.
//
// Usage: ./better_fastlio2 --lidar <topic> --imu <topic> --odometry <topic>
//        --registered_scan <topic> --global_map <topic> --corrected_path <topic>
//        [config args...]

#include "common_lib.h"
#include "preprocess.h"
#include "IMU_Processing.hpp"
#include "ikd_Tree.h"
#include "Scancontext.h"
#include "tgrs.h"

#include <omp.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <lcm/lcm-cpp.hpp>
#include "dimos_native_module.hpp"
#include "nav_msgs/Odometry.hpp"
#include "nav_msgs/Path.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

using namespace std;
using dimos::time_from_seconds;
using dimos::make_header;

// ============================================================================
// Constants
// ============================================================================
#define INIT_TIME       0.1
#define LASER_POINT_COV 0.001
// NUM_MATCH_POINTS defined in common_lib.h
// MP_PROC_NUM defined via CMake -DMP_PROC_NUM=N

// ============================================================================
// Global state
// ============================================================================
static atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;

// LCM topics (filled from CLI args)
static string g_lidar_topic, g_imu_topic;
static string g_odom_topic, g_scan_topic, g_map_topic, g_path_topic;

// Preprocessor
static shared_ptr<Preprocess> p_pre(new Preprocess());

// IMU processor
static shared_ptr<ImuProcess> p_imu(new ImuProcess());

// iKdtree
static KD_TREE<PointType> ikdtree;

// EKF
static esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
static state_ikfom state_point;
static V3D pos_lid;

// Scan Context
static ScanContext::SCManager scLoop;

// Dynamic removal
static TGRS remover;

// GTSAM
static gtsam::NonlinearFactorGraph gtSAMgraph;
static gtsam::Values initialEstimate;
static gtsam::Values isamCurrentEstimate;
static gtsam::ISAM2* isam = nullptr;

// Config
static int NUM_MAX_ITERATIONS = 3;
static double filter_size_map_min = 0.2;
static double cube_len = 1500.0;
static double fov_deg = 180.0;
static double DET_RANGE = 100.0;
static double FOV_DEG = 0, HALF_FOV_COS = 0;
static bool extrinsic_est_en = false;
static int kd_step = 30;
static bool recontructKdTree = false;
static float keyframeAddingDistThreshold = 1.0;
static float keyframeAddingAngleThreshold = 0.2;
static bool dense_pub_en = false;
static float publish_map_frequency = 0.0;

// Loop closure
static bool loopClosureEnableFlag = false;
static float loopClosureFrequency = 1.0;
static float historyKeyframeSearchRadius = 10.0;
static float historyKeyframeSearchTimeDiff = 30.0;
static int historyKeyframeSearchNum = 2;
static float historyKeyframeFitnessScore = 0.2;

// Segment
static bool ground_en = false;
static float sensor_height = 1.5;
static float z_tollerance = 2.0;
static float rotation_tollerance = 0.2;

// Dynamic removal
static bool dynamic_removal_enable = false;

// Buffers
static mutex mtx_buffer;
static condition_variable sig_buffer;
static deque<double> time_buffer;
static deque<pcl::PointCloud<PointType>::Ptr> lidar_buffer;
static deque<shared_ptr<ImuData>> imu_buffer;
static double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;

// Extrinsics
static vector<double> extrinT(3, 0.0);
static vector<double> extrinR(9, 0.0);
static V3D Lidar_T_wrt_IMU;
static M3D Lidar_R_wrt_IMU;

// IMU noise
static double gyr_cov = 0.1, acc_cov = 0.1;
static double b_gyr_cov = 0.0001, b_acc_cov = 0.0001;

// Processing state
static double first_lidar_time = 0.0;
static bool flg_first_scan = true, flg_EKF_inited = false;
static MeasureGroup Measures;
static pcl::PointCloud<PointType>::Ptr feats_undistort(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr feats_down_body(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr feats_down_world(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr normvec(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>(100000, 1));
static pcl::PointCloud<PointType>::Ptr corr_normvect(new pcl::PointCloud<PointType>(100000, 1));
static pcl::PointCloud<PointType>::Ptr featsFromMap(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr _featsArray;

static pcl::VoxelGrid<PointType> downSizeFilterSurf;
static pcl::VoxelGrid<PointType> downSizeFilterMap;
static pcl::VoxelGrid<PointType> downSizeFilterICP;

static int feats_down_size = 0;
static int effct_feat_num = 0;
static double total_residual = 0.0;
static double res_mean_last = 0.05;
static float res_last[100000] = {0.0};
static bool point_selected_surf[100000] = {0};
static double kdtree_incremental_time = 0.0;
static int add_point_size = 0;

static vector<BoxPointType> cub_needrm;
static vector<PointVector> Nearest_Points;
static vector<vector<int>> pointSearchInd_surf;

// Keyframe storage
static pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());
static vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

static float transformTobeMapped[6] = {0};

// Quaternion for publishing
static Eigen::Quaterniond geoQuat;
static V3D euler_cur;

// ============================================================================
// Forward declarations (implemented below, matching upstream logic)
// ============================================================================
static void pointBodyToWorld(const PointType* pi, PointType* po);
static void lasermap_fov_segment();
static void map_incremental();
static bool sync_packages(MeasureGroup& meas);
static void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);
static void publish_odometry(double timestamp);
static void publish_registered_scan(double timestamp);
static void publish_global_map(double timestamp);
static void publish_path(double timestamp);
static void getCurPose(const state_ikfom& s);
static void saveKeyFramesAndFactor();
static void correctPoses();

// ============================================================================
// Transform helpers (from upstream laserMapping.cpp)
// ============================================================================
static void pointBodyToWorld(const PointType* pi, PointType* po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// calc_dist is in common_lib.h

// ============================================================================
// getCurPose (from upstream)
// ============================================================================
static void getCurPose(const state_ikfom& s) {
    Eigen::Quaterniond q(s.rot.coeffs()[3], s.rot.coeffs()[0], s.rot.coeffs()[1], s.rot.coeffs()[2]);
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d euler = RotMtoEuler(R);
    transformTobeMapped[0] = euler(0);
    transformTobeMapped[1] = euler(1);
    transformTobeMapped[2] = euler(2);
    transformTobeMapped[3] = s.pos(0);
    transformTobeMapped[4] = s.pos(1);
    transformTobeMapped[5] = s.pos(2);
}

// ============================================================================
// Pose conversion helpers
// ============================================================================
static gtsam::Pose3 pclPointTogtsamPose3(const PointTypePose& p) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(double(p.roll), double(p.pitch), double(p.yaw)),
        gtsam::Point3(double(p.x), double(p.y), double(p.z)));
}

static gtsam::Pose3 trans2gtsamPose(float transformIn[]) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

static PointTypePose trans2PointTypePose(float transformIn[]) {
    PointTypePose p;
    p.x = transformIn[3];
    p.y = transformIn[4];
    p.z = transformIn[5];
    p.roll = transformIn[0];
    p.pitch = transformIn[1];
    p.yaw = transformIn[2];
    return p;
}

static pcl::PointCloud<PointType>::Ptr transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
        transformIn->x, transformIn->y, transformIn->z,
        transformIn->roll, transformIn->pitch, transformIn->yaw);

    #pragma omp parallel for num_threads(4)
    for (int i = 0; i < cloudSize; ++i) {
        const auto& pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y +
                                transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y +
                                transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y +
                                transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

// ============================================================================
// saveKeyFramesAndFactor (from upstream, simplified)
// ============================================================================
static mutex mtx_graph;  // dedicated mutex for GTSAM graph operations
static bool aLoopIsClosed = false;

static void saveKeyFramesAndFactor() {
    // Check if this frame qualifies as a keyframe
    if (cloudKeyPoses3D->points.empty()) {
        // First keyframe — tight prior (matches upstream)
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(
            0, trans2gtsamPose(transformTobeMapped),
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished())));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    } else {
        PointType lastPose = cloudKeyPoses3D->back();
        float dx = transformTobeMapped[3] - lastPose.x;
        float dy = transformTobeMapped[4] - lastPose.y;
        float dz = transformTobeMapped[5] - lastPose.z;
        float dist = sqrt(dx * dx + dy * dy + dz * dz);

        PointTypePose lastPose6D = cloudKeyPoses6D->back();
        float droll = fabs(transformTobeMapped[0] - lastPose6D.roll);
        float dpitch = fabs(transformTobeMapped[1] - lastPose6D.pitch);
        float dyaw = fabs(transformTobeMapped[2] - lastPose6D.yaw);
        float angle = droll + dpitch + dyaw;

        if (dist < keyframeAddingDistThreshold && angle < keyframeAddingAngleThreshold) {
            return;  // Not a keyframe
        }

        // Add odometry factor
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->back());
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),
            poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }

    // Optimize
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    isamCurrentEstimate = isam->calculateEstimate();

    // Save keyframe
    PointType thisPose3D;
    gtsam::Pose3 latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(
        isamCurrentEstimate.size() - 1);
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size();  // index
    cloudKeyPoses3D->push_back(thisPose3D);

    PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    thisPose6D.intensity = cloudKeyPoses6D->size();
    thisPose6D.time = Measures.lidar_beg_time;
    cloudKeyPoses6D->push_back(thisPose6D);

    // Save keyframe cloud (full undistorted, not downsampled — matches upstream)
    pcl::PointCloud<PointType>::Ptr thisCloud(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*feats_undistort, *thisCloud);
    surfCloudKeyFrames.push_back(thisCloud);

    // Save Scan Context
    scLoop.makeAndSaveScancontextAndKeys(*thisCloud);
}

// ============================================================================
// correctPoses (from upstream)
// ============================================================================
static void correctPoses() {
    if (cloudKeyPoses3D->points.empty()) return;
    if (!aLoopIsClosed) return;

    // Run extra ISAM updates to propagate loop closure (matches upstream)
    lock_guard<mutex> lock(mtx_graph);
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isamCurrentEstimate = isam->calculateEstimate();

    int numPoses = isamCurrentEstimate.size();
    for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 p = isamCurrentEstimate.at<gtsam::Pose3>(i);
        cloudKeyPoses3D->points[i].x = p.translation().x();
        cloudKeyPoses3D->points[i].y = p.translation().y();
        cloudKeyPoses3D->points[i].z = p.translation().z();
        cloudKeyPoses6D->points[i].x = p.translation().x();
        cloudKeyPoses6D->points[i].y = p.translation().y();
        cloudKeyPoses6D->points[i].z = p.translation().z();
        cloudKeyPoses6D->points[i].roll = p.rotation().roll();
        cloudKeyPoses6D->points[i].pitch = p.rotation().pitch();
        cloudKeyPoses6D->points[i].yaw = p.rotation().yaw();
    }

    // Feed corrected latest pose back to ESKF state
    gtsam::Pose3 latestPose = isamCurrentEstimate.at<gtsam::Pose3>(numPoses - 1);
    state_ikfom state_updated = kf.get_x();
    state_updated.pos = V3D(latestPose.translation().x(),
                            latestPose.translation().y(),
                            latestPose.translation().z());
    state_updated.rot = Eigen::Quaterniond(latestPose.rotation().matrix()).normalized();
    kf.change_x(state_updated);

    aLoopIsClosed = false;
}

// ============================================================================
// lasermap_fov_segment (from upstream, faithful FOV-based local map management)
// ============================================================================
static bool Localmap_Initialized = false;
static BoxPointType LocalMap_Points;
static int kdtree_delete_counter = 0;

static void lasermap_fov_segment() {
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    V3D pos_LiD = pos_lid;

    // Initialize local map bounding box centered on lidar position
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    // Check distance to each map boundary face
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            need_move = true;
        }
    }
    if (!need_move) return;

    // Compute new bounding box and boxes to remove
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    // Delete points in removed boxes from ikdtree
    if (cub_needrm.size() > 0) {
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    }
}

// ============================================================================
// map_incremental (from upstream, faithful)
// ============================================================================
static void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        if (!Nearest_Points[i].empty() && flg_EKF_inited) {
            const PointVector& points_near = Nearest_Points[i];
            bool need_add = true;
            PointType mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);

            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }

            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
}

// ============================================================================
// h_share_model (from upstream, faithful — measurement model for iESKF)
// ============================================================================
static void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) {
        PointType& point_body = feats_down_body->points[i];
        PointType& point_world = feats_down_world->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto& points_near = Nearest_Points[i];

        if (ekfom_data.converge) {
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false
                : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                         pabcd(2) * point_world.z + pabcd(3);
            float s_val = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            if (s_val > 0.9) {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        fprintf(stderr, "[better_fastlio2] No effective points!\n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;

    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
        const PointType& laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType& norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);

        if (extrinsic_est_en) {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        ekfom_data.h(i) = -norm_p.intensity;
    }
}

// ============================================================================
// sync_packages (from upstream, adapted for LCM buffers)
// ============================================================================
static bool sync_packages(MeasureGroup& meas) {
    if (lidar_buffer.empty() || imu_buffer.empty()) return false;

    if (!meas.lidar) {
        meas.lidar.reset(new pcl::PointCloud<PointType>());
    }

    // Get lidar scan
    *(meas.lidar) = *(lidar_buffer.front());
    meas.lidar_beg_time = time_buffer.front();

    double lidar_end_time;
    if (meas.lidar->points.size() <= 1) {
        lidar_end_time = meas.lidar_beg_time + 0.1;
    } else {
        lidar_end_time = meas.lidar_beg_time +
                         meas.lidar->points.back().curvature / 1000.0;
    }
    meas.lidar_end_time = lidar_end_time;

    // Check if we have enough IMU data
    if (last_timestamp_imu < lidar_end_time) return false;

    // Collect IMU data for this lidar frame
    meas.imu.clear();
    while (!imu_buffer.empty()) {
        double imu_time = imu_buffer.front()->timestamp;
        if (imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    return true;
}

// ============================================================================
// LCM publish helpers
// ============================================================================
static void publish_pointcloud_lcm(const string& topic, pcl::PointCloud<PointType>::Ptr cloud,
                                    double timestamp, const string& frame_id = "map") {
    if (!g_lcm || cloud->empty() || topic.empty()) return;

    int num_points = cloud->size();
    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(frame_id, timestamp);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    pc.fields_length = 4;
    pc.fields.resize(4);
    auto make_field = [](const string& name, int32_t offset) {
        sensor_msgs::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = sensor_msgs::PointField::FLOAT32;
        f.count = 1;
        return f;
    };
    pc.fields[0] = make_field("x", 0);
    pc.fields[1] = make_field("y", 4);
    pc.fields[2] = make_field("z", 8);
    pc.fields[3] = make_field("intensity", 12);
    pc.point_step = 16;
    pc.row_step = pc.point_step * num_points;
    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);

    for (int i = 0; i < num_points; ++i) {
        float* dst = reinterpret_cast<float*>(pc.data.data() + i * 16);
        dst[0] = cloud->points[i].x;
        dst[1] = cloud->points[i].y;
        dst[2] = cloud->points[i].z;
        dst[3] = cloud->points[i].intensity;
    }

    g_lcm->publish(topic, &pc);
}

static void publish_odometry(double timestamp) {
    if (!g_lcm || g_odom_topic.empty()) return;

    nav_msgs::Odometry odom;
    odom.header = make_header("map", timestamp);
    odom.child_frame_id = "body";

    odom.pose.pose.position.x = state_point.pos(0);
    odom.pose.pose.position.y = state_point.pos(1);
    odom.pose.pose.position.z = state_point.pos(2);
    odom.pose.pose.orientation.x = geoQuat.x();
    odom.pose.pose.orientation.y = geoQuat.y();
    odom.pose.pose.orientation.z = geoQuat.z();
    odom.pose.pose.orientation.w = geoQuat.w();

    odom.twist.twist.linear.x = state_point.vel(0);
    odom.twist.twist.linear.y = state_point.vel(1);
    odom.twist.twist.linear.z = state_point.vel(2);

    g_lcm->publish(g_odom_topic, &odom);
}

static void publish_registered_scan(double timestamp) {
    if (g_scan_topic.empty()) return;

    pcl::PointCloud<PointType>::Ptr laserCloudWorld(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr source = dense_pub_en ? feats_undistort : feats_down_body;
    int size = source->size();
    laserCloudWorld->resize(size);

    for (int i = 0; i < size; i++) {
        pointBodyToWorld(&source->points[i], &laserCloudWorld->points[i]);
    }

    publish_pointcloud_lcm(g_scan_topic, laserCloudWorld, timestamp);
}

static void publish_global_map(double timestamp) {
    if (g_map_topic.empty() || cloudKeyPoses3D->empty()) return;

    pcl::PointCloud<PointType>::Ptr globalMap(new pcl::PointCloud<PointType>());
    for (size_t i = 0; i < surfCloudKeyFrames.size(); ++i) {
        *globalMap += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
    }

    pcl::VoxelGrid<PointType> ds;
    ds.setLeafSize(0.2f, 0.2f, 0.2f);
    ds.setInputCloud(globalMap);
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
    ds.filter(*filtered);

    publish_pointcloud_lcm(g_map_topic, filtered, timestamp);
}

static void publish_path(double timestamp) {
    // Path publishing via LCM (nav_msgs::Path)
    // TODO: implement if needed for visualization
    (void)timestamp;
}

// ============================================================================
// LCM callbacks
// ============================================================================
static void on_lidar(const lcm::ReceiveBuffer* /*rbuf*/, const string& /*channel*/,
                     const sensor_msgs::PointCloud2* msg) {
    if (!g_running.load()) return;

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    // Convert LCM PointCloud2 to PCL
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

    int num_points = msg->width * msg->height;
    if (num_points == 0) return;

    // Parse fields to find offsets
    int x_off = -1, y_off = -1, z_off = -1, int_off = -1;
    int time_off = -1, ring_off = -1, tag_off = -1;
    for (const auto& f : msg->fields) {
        if (f.name == "x") x_off = f.offset;
        else if (f.name == "y") y_off = f.offset;
        else if (f.name == "z") z_off = f.offset;
        else if (f.name == "intensity") int_off = f.offset;
        else if (f.name == "time" || f.name == "offset_time" || f.name == "t") time_off = f.offset;
        else if (f.name == "ring") ring_off = f.offset;
        else if (f.name == "tag") tag_off = f.offset;
    }

    if (x_off < 0 || y_off < 0 || z_off < 0) return;

    // Build PCL cloud with preprocessing
    cloud->resize(num_points);
    for (int i = 0; i < num_points; ++i) {
        const uint8_t* ptr = msg->data.data() + i * msg->point_step;
        PointType& pt = cloud->points[i];
        pt.x = *reinterpret_cast<const float*>(ptr + x_off);
        pt.y = *reinterpret_cast<const float*>(ptr + y_off);
        pt.z = *reinterpret_cast<const float*>(ptr + z_off);
        pt.intensity = int_off >= 0 ? *reinterpret_cast<const float*>(ptr + int_off) : 0.0f;
        pt.normal_x = 0;
        pt.normal_y = 0;
        pt.normal_z = 0;

        // Per-point timestamp offset in milliseconds (stored in curvature)
        if (time_off >= 0) {
            // Check field type from the name hint
            // offset_time is uint32 in nanoseconds (from Mid360)
            // time is float32 in seconds or ms (from Velodyne)
            // t is uint32 in nanoseconds (from Ouster)
            for (const auto& f : msg->fields) {
                if (f.offset == time_off) {
                    if (f.datatype == sensor_msgs::PointField::UINT32) {
                        uint32_t t = *reinterpret_cast<const uint32_t*>(ptr + time_off);
                        pt.curvature = static_cast<float>(t) / 1e6f;  // ns → ms
                    } else {
                        pt.curvature = *reinterpret_cast<const float*>(ptr + time_off);
                        // Scale based on p_pre->time_unit if needed
                    }
                    break;
                }
            }
        } else {
            pt.curvature = 0.0f;
        }
    }

    // Apply preprocessing (blind zone filter, downsampling)
    pcl::PointCloud<PointType>::Ptr processed(new pcl::PointCloud<PointType>());
    int plsize = cloud->size();
    processed->reserve(plsize);
    int valid_num = 0;
    double blind = p_pre->blind;
    int pfn = p_pre->point_filter_num;

    for (int i = 0; i < plsize; i++) {
        const auto& pt = cloud->points[i];
        float dist2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (dist2 < blind * blind) continue;
        valid_num++;
        if (valid_num % pfn == 0) {
            processed->push_back(pt);
        }
    }

    lock_guard<mutex> lock(mtx_buffer);
    if (timestamp < last_timestamp_lidar) {
        fprintf(stderr, "[better_fastlio2] lidar loop detected, clearing buffer\n");
        lidar_buffer.clear();
        time_buffer.clear();
    }
    last_timestamp_lidar = timestamp;
    lidar_buffer.push_back(processed);
    time_buffer.push_back(timestamp);
    sig_buffer.notify_all();
}

static void on_imu(const lcm::ReceiveBuffer* /*rbuf*/, const string& /*channel*/,
                   const sensor_msgs::Imu* msg) {
    if (!g_running.load()) return;

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    auto imu = make_shared<ImuData>();
    imu->timestamp = timestamp;
    imu->acc = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    imu->gyro = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    lock_guard<mutex> lock(mtx_buffer);
    if (timestamp < last_timestamp_imu) {
        fprintf(stderr, "[better_fastlio2] imu loop detected, clearing buffer\n");
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(imu);
    sig_buffer.notify_all();
}

// ============================================================================
// Loop closure thread (from upstream)
// ============================================================================
static void loopClosureThread() {
    if (!loopClosureEnableFlag) return;

    auto sleep_time = chrono::milliseconds(static_cast<int>(1000.0 / loopClosureFrequency));
    while (g_running.load()) {
        this_thread::sleep_for(sleep_time);

        // Scan Context loop closure detection
        if (cloudKeyPoses3D->points.size() < 50) continue;

        auto [loopIdx, yawDiffRad] = scLoop.detectLoopClosureID();
        if (loopIdx == -1) continue;

        // ICP verification
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());

        int curIdx = cloudKeyPoses3D->size() - 1;
        *cureKeyframeCloud = *transformPointCloud(surfCloudKeyFrames[curIdx], &cloudKeyPoses6D->points[curIdx]);
        *prevKeyframeCloud = *transformPointCloud(surfCloudKeyFrames[loopIdx], &cloudKeyPoses6D->points[loopIdx]);

        pcl::VoxelGrid<PointType> ds;
        ds.setLeafSize(0.2f, 0.2f, 0.2f);
        ds.setInputCloud(cureKeyframeCloud);
        ds.filter(*cureKeyframeCloud);
        ds.setInputCloud(prevKeyframeCloud);
        ds.filter(*prevKeyframeCloud);

        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);

        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (!icp.hasConverged() || icp.getFitnessScore() > historyKeyframeFitnessScore) {
            continue;
        }

        printf("[better_fastlio2] Loop closure detected: %d → %d (fitness: %.3f)\n",
               curIdx, loopIdx, icp.getFitnessScore());

        // Add loop factor to graph
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                                              gtsam::Point3(x, y, z));
        gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3());

        auto robustNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()));

        lock_guard<mutex> lock(mtx_graph);
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            curIdx, loopIdx, poseFrom.between(poseTo), robustNoise));
        aLoopIsClosed = true;
    }
}

// ============================================================================
// Signal handling
// ============================================================================
static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // Required topics
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_imu_topic = mod.has("imu") ? mod.topic("imu") : "";
    g_odom_topic = mod.has("odometry") ? mod.topic("odometry") : "";
    g_scan_topic = mod.has("registered_scan") ? mod.topic("registered_scan") : "";
    g_map_topic = mod.has("global_map") ? mod.topic("global_map") : "";
    g_path_topic = mod.has("corrected_path") ? mod.topic("corrected_path") : "";

    if (g_lidar_topic.empty() || g_imu_topic.empty()) {
        fprintf(stderr, "Error: --lidar and --imu topics are required\n");
        return 1;
    }

    // Config from CLI args
    p_pre->lidar_type = mod.arg_int("lidar_type", 2);
    p_pre->N_SCANS = mod.arg_int("scan_line", 16);
    p_pre->blind = mod.arg_float("blind", 2.0);
    p_pre->feature_enabled = mod.arg("feature_enabled", "false") == "true";
    p_pre->point_filter_num = mod.arg_int("point_filter_num", 4);
    p_pre->SCAN_RATE = mod.arg_int("scan_rate", 10);

    acc_cov = mod.arg_float("acc_cov", 0.1);
    gyr_cov = mod.arg_float("gyr_cov", 0.1);
    b_acc_cov = mod.arg_float("b_acc_cov", 0.0001);
    b_gyr_cov = mod.arg_float("b_gyr_cov", 0.0001);
    NUM_MAX_ITERATIONS = mod.arg_int("max_iteration", 3);
    extrinsic_est_en = mod.arg("extrinsic_est_en", "false") == "true";
    DET_RANGE = mod.arg_float("det_range", 100.0);
    fov_deg = mod.arg_float("fov_degree", 180.0);
    cube_len = mod.arg_float("cube_len", 1500.0);
    filter_size_map_min = mod.arg_float("filter_size_map_min", 0.2);
    keyframeAddingDistThreshold = mod.arg_float("keyframe_dist_threshold", 1.0);
    keyframeAddingAngleThreshold = mod.arg_float("keyframe_angle_threshold", 0.2);
    kd_step = mod.arg_int("kd_step", 30);
    recontructKdTree = mod.arg("reconstruct_kdtree", "false") == "true";
    dense_pub_en = mod.arg("dense_publish_en", "false") == "true";
    publish_map_frequency = mod.arg_float("publish_map_frequency", 0.0);

    // Loop closure config
    loopClosureEnableFlag = mod.arg("loop_closure_enable", "false") == "true";
    loopClosureFrequency = mod.arg_float("loop_closure_frequency", 1.0);
    historyKeyframeSearchRadius = mod.arg_float("history_keyframe_search_radius", 10.0);
    historyKeyframeSearchTimeDiff = mod.arg_float("history_keyframe_search_time_diff", 30.0);
    historyKeyframeSearchNum = mod.arg_int("history_keyframe_search_num", 2);
    historyKeyframeFitnessScore = mod.arg_float("history_keyframe_fitness_score", 0.2);

    // Dynamic removal
    dynamic_removal_enable = mod.arg("dynamic_removal_enable", "false") == "true";

    // Segment
    ground_en = mod.arg("ground_en", "false") == "true";
    sensor_height = mod.arg_float("sensor_height", 1.5);
    z_tollerance = mod.arg_float("z_tolerance", 2.0);
    rotation_tollerance = mod.arg_float("rotation_tolerance", 0.2);

    // Extrinsics (parse comma-separated list)
    string ext_t_str = mod.arg("extrinsic_T", "0,0,0");
    string ext_r_str = mod.arg("extrinsic_R", "1,0,0,0,1,0,0,0,1");
    // Parse extrinsic_T
    {
        stringstream ss(ext_t_str);
        string item;
        int idx = 0;
        while (getline(ss, item, ',') && idx < 3) {
            extrinT[idx++] = stod(item);
        }
    }
    // Parse extrinsic_R
    {
        stringstream ss(ext_r_str);
        string item;
        int idx = 0;
        while (getline(ss, item, ',') && idx < 9) {
            extrinR[idx++] = stod(item);
        }
    }

    printf("[better_fastlio2] Starting enhanced FAST-LIO2 module\n");
    printf("[better_fastlio2] lidar: %s | imu: %s | odom: %s\n",
           g_lidar_topic.c_str(), g_imu_topic.c_str(), g_odom_topic.c_str());
    printf("[better_fastlio2] lidar_type=%d scan_line=%d blind=%.1f\n",
           p_pre->lidar_type, p_pre->N_SCANS, p_pre->blind);
    printf("[better_fastlio2] loop_closure=%s dynamic_removal=%s\n",
           loopClosureEnableFlag ? "on" : "off",
           dynamic_removal_enable ? "on" : "off");

    // Signal handlers
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    // Init LCM
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    // Subscribe to inputs
    lcm::LCM::HandlerFunction<sensor_msgs::PointCloud2> lidar_handler = on_lidar;
    lcm::LCM::HandlerFunction<sensor_msgs::Imu> imu_handler = on_imu;
    lcm.subscribe(g_lidar_topic, lidar_handler);
    lcm.subscribe(g_imu_topic, imu_handler);

    // Init GTSAM
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    // Init EKF
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * M_PI / 180.0);

    _featsArray.reset(new pcl::PointCloud<PointType>());
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    float mappingSurfLeafSize = filter_size_map_min;
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

    // Set IMU/lidar extrinsics
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // Init EKF with process/measurement models
    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    // Start loop closure thread
    thread loopthread(loopClosureThread);

    printf("[better_fastlio2] Initialized. Waiting for data...\n");

    // Main processing loop
    int frame_count = 0;
    auto last_map_publish = chrono::steady_clock::now();

    while (g_running.load()) {
        lcm.handleTimeout(10);

        lock_guard<mutex> lock(mtx_buffer);

        if (!sync_packages(Measures)) continue;

        // First scan
        if (flg_first_scan) {
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            continue;
        }

        // IMU processing: forward propagation + undistortion
        p_imu->Process(Measures, kf, feats_undistort);
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

        if (feats_undistort->empty()) {
            fprintf(stderr, "[better_fastlio2] No undistorted points, skip\n");
            continue;
        }

        // Dynamic object removal (T-GRS 2024)
        if (dynamic_removal_enable && feats_undistort->points.size() > 0) {
            SSC ssc_cur(feats_undistort, 0);
            remover.cluster(ssc_cur.apri_vec, ssc_cur.hash_cloud, ssc_cur.cluster_vox);
            remover.recognizePD(ssc_cur);
            // Replace undistorted cloud with non-dynamic points
            feats_undistort->points.clear();
            *feats_undistort += *ssc_cur.cloud_nd;
        }

        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

        // FOV segmentation
        lasermap_fov_segment();

        // Downsample
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        feats_down_size = feats_down_body->points.size();

        // Initialize iKdtree
        if (ikdtree.Root_Node == nullptr) {
            if (feats_down_size > 5) {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++) {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                ikdtree.Build(feats_down_world->points);
            }
            continue;
        }

        if (feats_down_size < 5) continue;

        // Prepare for ICP
        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);
        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);

        // Iterated EKF update
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point = kf.get_x();
        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        geoQuat.x() = state_point.rot.coeffs()[0];
        geoQuat.y() = state_point.rot.coeffs()[1];
        geoQuat.z() = state_point.rot.coeffs()[2];
        geoQuat.w() = state_point.rot.coeffs()[3];

        // Backend: keyframes + factor graph
        getCurPose(state_point);
        saveKeyFramesAndFactor();
        correctPoses();

        // Publish odometry
        publish_odometry(Measures.lidar_end_time);

        // Map incremental update
        map_incremental();

        // Publish registered scan
        publish_registered_scan(Measures.lidar_end_time);

        // Periodic global map publish
        if (publish_map_frequency > 0) {
            auto now = chrono::steady_clock::now();
            double elapsed = chrono::duration<double>(now - last_map_publish).count();
            if (elapsed >= 1.0 / publish_map_frequency) {
                publish_global_map(Measures.lidar_end_time);
                last_map_publish = now;
            }
        }

        // Periodic iKdtree reconstruction
        frame_count++;
        if (recontructKdTree && kd_step > 0 && frame_count % kd_step == 0) {
            PointVector all_points;
            ikdtree.flatten(ikdtree.Root_Node, all_points, NOT_RECORD);
            ikdtree.reconstruct(all_points);
            printf("[better_fastlio2] iKdtree reconstructed at frame %d\n", frame_count);
        }
    }

    // Cleanup
    printf("[better_fastlio2] Shutting down...\n");
    g_running.store(false);
    loopthread.join();
    delete isam;
    g_lcm = nullptr;

    printf("[better_fastlio2] Done.\n");
    return 0;
}
