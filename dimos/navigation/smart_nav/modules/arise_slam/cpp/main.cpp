// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// arise_slam native module for dimos NativeModule framework.
//
// LiDAR-Inertial SLAM combining feature extraction, laser mapping (ICP/Ceres),
// and IMU preintegration (GTSAM iSAM2) into a single LCM-based process.
//
// Usage: ./arise_slam --lidar <topic> --imu <topic> --odometry <topic>
//        --registered_scan <topic> [config args...]

#include "arise_slam_mid360/FeatureExtraction/featureExtraction.h"
#include "arise_slam_mid360/LaserMapping/laserMapping.h"
#include "arise_slam_mid360/ImuPreintegration/imuPreintegration.h"
#include "arise_slam_mid360/config/parameter.h"
#include "arise_slam_mid360/sensor_data/pointcloud/point_os.h"
#include "arise_slam_mid360/common.h"

#include <lcm/lcm-cpp.hpp>
#include "dimos_native_module.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "geometry_msgs/Quaternion.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <mutex>
#include <string>
#include <vector>

using namespace std;
using dimos::time_from_seconds;
using dimos::make_header;

// ============================================================================
// Global state
// ============================================================================
static atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;

// LCM topics
static string g_lidar_topic, g_imu_topic;
static string g_odom_topic, g_scan_topic;

// Algorithm components
static arise_slam::featureExtraction g_feature_extraction;
static arise_slam::laserMapping g_laser_mapping;
static arise_slam::imuPreintegration g_imu_preintegration;

// Processing state
static mutex g_process_mtx;
static bool g_has_imu_preintegration = true;  // can be disabled via config

// Frame counter for logging
static int g_frame_count = 0;

// ============================================================================
// Signal handling
// ============================================================================
static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

// ============================================================================
// PointCloud2 → PCL conversion
// ============================================================================
static pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr
convertPointCloud2ToPCL(const sensor_msgs::PointCloud2* msg, double timestamp) {
    auto cloud = pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr(
        new pcl::PointCloud<point_os::PointcloudXYZITR>());

    int num_points = msg->width * msg->height;
    if (num_points == 0) {
        return cloud;
    }

    // Parse field offsets
    int x_off = -1, y_off = -1, z_off = -1, int_off = -1;
    int time_off = -1, ring_off = -1;
    uint8_t time_datatype = 0;
    for (const auto& f : msg->fields) {
        if (f.name == "x") { x_off = f.offset; }
        else if (f.name == "y") { y_off = f.offset; }
        else if (f.name == "z") { z_off = f.offset; }
        else if (f.name == "intensity") { int_off = f.offset; }
        else if (f.name == "time" || f.name == "offset_time" || f.name == "t") {
            time_off = f.offset;
            time_datatype = f.datatype;
        }
        else if (f.name == "ring") { ring_off = f.offset; }
    }

    if (x_off < 0 || y_off < 0 || z_off < 0) {
        fprintf(stderr, "[arise_slam] PointCloud2 missing x/y/z fields\n");
        return cloud;
    }

    cloud->resize(num_points);
    for (int i = 0; i < num_points; ++i) {
        const uint8_t* ptr = msg->data.data() + i * msg->point_step;
        auto& pt = cloud->points[i];

        pt.x = *reinterpret_cast<const float*>(ptr + x_off);
        pt.y = *reinterpret_cast<const float*>(ptr + y_off);
        pt.z = *reinterpret_cast<const float*>(ptr + z_off);
        pt.intensity = int_off >= 0 ? *reinterpret_cast<const float*>(ptr + int_off) : 0.0f;

        // Per-point timestamp
        if (time_off >= 0) {
            if (time_datatype == sensor_msgs::PointField::UINT32) {
                // offset_time / t: nanoseconds as uint32
                uint32_t t_ns = *reinterpret_cast<const uint32_t*>(ptr + time_off);
                pt.time = static_cast<float>(t_ns) * 1e-9f;  // ns → seconds
            } else {
                // time: float seconds (or ms depending on sensor)
                pt.time = *reinterpret_cast<const float*>(ptr + time_off);
            }
        } else {
            pt.time = 0.0f;
        }

        // Ring/scan line
        if (ring_off >= 0) {
            pt.ring = *reinterpret_cast<const uint16_t*>(ptr + ring_off);
        } else {
            pt.ring = 0;
        }
    }

    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

// ============================================================================
// LCM publish: odometry
// ============================================================================
static void publish_odometry(const arise_slam::ImuState& state) {
    if (!g_lcm || g_odom_topic.empty()) {
        return;
    }

    nav_msgs::Odometry odom;
    odom.header = make_header("map", state.timestamp);
    odom.child_frame_id = "body";

    odom.pose.pose.position.x = state.position.x();
    odom.pose.pose.position.y = state.position.y();
    odom.pose.pose.position.z = state.position.z();
    odom.pose.pose.orientation.x = state.orientation.x();
    odom.pose.pose.orientation.y = state.orientation.y();
    odom.pose.pose.orientation.z = state.orientation.z();
    odom.pose.pose.orientation.w = state.orientation.w();

    odom.twist.twist.linear.x = state.velocity.x();
    odom.twist.twist.linear.y = state.velocity.y();
    odom.twist.twist.linear.z = state.velocity.z();
    odom.twist.twist.angular.x = state.angular_velocity.x();
    odom.twist.twist.angular.y = state.angular_velocity.y();
    odom.twist.twist.angular.z = state.angular_velocity.z();

    // Store health/result info in covariance
    odom.pose.covariance[0] = state.health_status ? 1.0 : 0.0;
    odom.pose.covariance[1] = static_cast<double>(state.result);
    odom.pose.covariance[2] = state.gravity;

    g_lcm->publish(g_odom_topic, &odom);
}

// Fallback: publish odometry from laser mapping output (no IMU preintegration)
static void publish_odometry_from_mapping(const arise_slam::LaserMappingOutput& output) {
    if (!g_lcm || g_odom_topic.empty()) {
        return;
    }

    nav_msgs::Odometry odom;
    odom.header = make_header("map", output.timestamp);
    odom.child_frame_id = "body";

    odom.pose.pose.position.x = output.position.x();
    odom.pose.pose.position.y = output.position.y();
    odom.pose.pose.position.z = output.position.z();
    odom.pose.pose.orientation.x = output.orientation.x();
    odom.pose.pose.orientation.y = output.orientation.y();
    odom.pose.pose.orientation.z = output.orientation.z();
    odom.pose.pose.orientation.w = output.orientation.w();

    odom.twist.twist.linear.x = output.velocity.x();
    odom.twist.twist.linear.y = output.velocity.y();
    odom.twist.twist.linear.z = output.velocity.z();

    // Flag degeneracy in covariance
    odom.pose.covariance[0] = output.is_degenerate ? 0.0 : 1.0;

    g_lcm->publish(g_odom_topic, &odom);
}

// ============================================================================
// LCM publish: registered scan
// ============================================================================
static void publish_registered_scan(const pcl::PointCloud<PointType>::Ptr& cloud,
                                     double timestamp) {
    if (!g_lcm || g_scan_topic.empty() || !cloud || cloud->empty()) {
        return;
    }

    int num_points = cloud->size();

    sensor_msgs::PointCloud2 pc;
    pc.header = make_header("map", timestamp);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    // Fields: x, y, z, intensity
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

    g_lcm->publish(g_scan_topic, &pc);
}

// ============================================================================
// Processing pipeline (triggered by lidar callback)
// ============================================================================
static void run_pipeline() {
    // Step 1: Run feature extraction undistortion + scan registration
    g_feature_extraction.undistortionAndscanregistration();

    // Step 2: Get latest feature extraction result
    arise_slam::FeatureExtractionResult fe_result = g_feature_extraction.getLatestResult();
    if (!fe_result.valid) {
        return;
    }

    g_frame_count++;

    // Step 3: Feed features to laser mapping
    // The realsense cloud is nullptr if not used
    pcl::PointCloud<PointType>::Ptr realsense_cloud(new pcl::PointCloud<PointType>());
    g_laser_mapping.addLaserFeature(
        fe_result.edgePoints,
        fe_result.plannerPoints,
        fe_result.depthPoints,  // full cloud / depth points
        realsense_cloud,
        fe_result.q_w_original_l,
        fe_result.timestamp
    );

    // Step 4: Run laser mapping optimization
    g_laser_mapping.process();

    // Step 5: Get mapping output
    arise_slam::LaserMappingOutput mapping_output = g_laser_mapping.getLatestOutput();

    // Step 6: Feed laser odometry to IMU preintegration
    if (g_has_imu_preintegration) {
        double degenerate_flag = mapping_output.is_degenerate ? 1.0 : 0.0;
        g_imu_preintegration.addLaserOdometry(
            mapping_output.timestamp,
            mapping_output.position,
            mapping_output.orientation,
            degenerate_flag
        );

        // Step 7: Get IMU-fused state and publish
        arise_slam::ImuState imu_state = g_imu_preintegration.getLatestState();
        publish_odometry(imu_state);
    } else {
        // Fallback: publish laser mapping odometry directly
        publish_odometry_from_mapping(mapping_output);
    }

    // Step 8: Publish registered scan
    if (mapping_output.registered_scan && !mapping_output.registered_scan->empty()) {
        publish_registered_scan(mapping_output.registered_scan, mapping_output.timestamp);
    }

    if (g_frame_count % 100 == 0) {
        printf("[arise_slam] Processed %d frames, pos=(%.2f, %.2f, %.2f)\n",
               g_frame_count,
               mapping_output.position.x(),
               mapping_output.position.y(),
               mapping_output.position.z());
    }
}

// ============================================================================
// LCM callbacks
// ============================================================================
static void on_lidar(const lcm::ReceiveBuffer* /*rbuf*/, const string& /*channel*/,
                     const sensor_msgs::PointCloud2* msg) {
    if (!g_running.load()) {
        return;
    }

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    // Convert LCM PointCloud2 → PCL PointcloudXYZITR
    auto cloud = convertPointCloud2ToPCL(msg, timestamp);
    if (cloud->empty()) {
        return;
    }

    // Feed point cloud to feature extraction
    g_feature_extraction.addPointCloud(timestamp, cloud);

    // Run the full processing pipeline
    lock_guard<mutex> lock(g_process_mtx);
    run_pipeline();
}

static void on_imu(const lcm::ReceiveBuffer* /*rbuf*/, const string& /*channel*/,
                   const sensor_msgs::Imu* msg) {
    if (!g_running.load()) {
        return;
    }

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    Eigen::Vector3d acc(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
    Eigen::Vector3d gyr(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
    Eigen::Quaterniond orientation(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    );

    // Check if orientation data is valid (non-zero quaternion)
    bool has_orientation = (orientation.squaredNorm() > 0.5);

    // Feed IMU to feature extraction (for deskewing)
    g_feature_extraction.addImuData(timestamp, acc, gyr, orientation, has_orientation);

    // Feed IMU to preintegration (for factor graph)
    if (g_has_imu_preintegration) {
        g_imu_preintegration.addImuMeasurement(timestamp, acc, gyr, orientation);
    }
}

// ============================================================================
// Parse sensor type from string
// ============================================================================
static SensorType parseSensorType(const string& s) {
    if (s == "velodyne") { return SensorType::VELODYNE; }
    if (s == "ouster") { return SensorType::OUSTER; }
    if (s == "livox") { return SensorType::LIVOX; }
    // Default to livox
    fprintf(stderr, "[arise_slam] Unknown sensor type '%s', defaulting to livox\n", s.c_str());
    return SensorType::LIVOX;
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // ---- Required topics ----
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_imu_topic = mod.has("imu") ? mod.topic("imu") : "";
    g_odom_topic = mod.has("odometry") ? mod.topic("odometry") : "";
    g_scan_topic = mod.has("registered_scan") ? mod.topic("registered_scan") : "";

    if (g_lidar_topic.empty() || g_imu_topic.empty()) {
        fprintf(stderr, "Error: --lidar and --imu topics are required\n");
        return 1;
    }

    // ---- Parse shared config ----
    string sensor_str = mod.arg("sensor", "livox");
    SensorType sensor_type = parseSensorType(sensor_str);
    bool use_imu_roll_pitch = mod.arg("use_imu_roll_pitch", "true") == "true";
    bool lidar_flip = mod.arg("lidar_flip", "false") == "true";

    // IMU offset/limit params (shared by feature extraction and IMU preintegration)
    double imu_acc_x_offset = mod.arg_float("imu_acc_x_offset", 0.0);
    double imu_acc_y_offset = mod.arg_float("imu_acc_y_offset", 0.0);
    double imu_acc_z_offset = mod.arg_float("imu_acc_z_offset", 0.0);
    double imu_acc_x_limit = mod.arg_float("imu_acc_x_limit", 100.0);
    double imu_acc_y_limit = mod.arg_float("imu_acc_y_limit", 100.0);
    double imu_acc_z_limit = mod.arg_float("imu_acc_z_limit", 100.0);

    // ---- Feature extraction config ----
    arise_slam::feature_extraction_config fe_config;
    fe_config.box_size.blindFront = mod.arg_float("blind_front", 0.5);
    fe_config.box_size.blindBack = mod.arg_float("blind_back", 0.5);
    fe_config.box_size.blindRight = mod.arg_float("blind_right", 0.5);
    fe_config.box_size.blindLeft = mod.arg_float("blind_left", 0.5);
    fe_config.box_size.blindDiskLow = mod.arg_float("blind_disk_low", -0.3);
    fe_config.box_size.blindDiskHigh = mod.arg_float("blind_disk_high", 0.3);
    fe_config.box_size.blindDiskRadius = mod.arg_float("blind_disk_radius", 0.5);
    fe_config.skipFrame = mod.arg_int("skip_frame", 0);
    fe_config.lidar_flip = lidar_flip;
    fe_config.N_SCANS = mod.arg_int("scan_line", 4);
    fe_config.provide_point_time = mod.arg_int("provide_point_time", 1);
    fe_config.point_filter_num = mod.arg_int("point_filter_num", 1);
    fe_config.use_dynamic_mask = mod.arg("use_dynamic_mask", "false") == "true";
    fe_config.use_imu_roll_pitch = use_imu_roll_pitch;
    fe_config.use_up_realsense_points = mod.arg("use_up_realsense_points", "false") == "true";
    fe_config.use_down_realsense_points = mod.arg("use_down_realsense_points", "false") == "true";
    fe_config.debug_view_enabled = mod.arg("debug_view_enabled", "false") == "true";
    fe_config.min_range = mod.arg_float("min_range", 0.3);
    fe_config.max_range = mod.arg_float("max_range", 100.0);
    fe_config.skip_realsense_points = mod.arg_int("skip_realsense_points", 1);
    fe_config.sensor = sensor_type;
    fe_config.livox_pitch = mod.arg_float("livox_pitch", 0.0);
    fe_config.imu_acc_x_offset = imu_acc_x_offset;
    fe_config.imu_acc_y_offset = imu_acc_y_offset;
    fe_config.imu_acc_z_offset = imu_acc_z_offset;
    fe_config.imu_acc_x_limit = imu_acc_x_limit;
    fe_config.imu_acc_y_limit = imu_acc_y_limit;
    fe_config.imu_acc_z_limit = imu_acc_z_limit;

    // ---- Laser mapping config ----
    arise_slam::laser_mapping_config lm_config;
    lm_config.period = mod.arg_float("mapping_period", 0.1);
    lm_config.lineRes = mod.arg_float("mapping_line_resolution", 0.2);
    lm_config.planeRes = mod.arg_float("mapping_plane_resolution", 0.4);
    lm_config.max_iterations = mod.arg_int("max_iterations", 10);
    lm_config.debug_view_enabled = mod.arg("debug_view_enabled", "false") == "true";
    lm_config.enable_ouster_data = (sensor_type == SensorType::OUSTER);
    lm_config.publish_only_feature_points = mod.arg("publish_only_feature_points", "false") == "true";
    lm_config.use_imu_roll_pitch = use_imu_roll_pitch;
    lm_config.max_surface_features = mod.arg_int("max_surface_features", 500);
    lm_config.velocity_failure_threshold = mod.arg_float("velocity_failure_threshold", 20.0);
    lm_config.auto_voxel_size = mod.arg("auto_voxel_size", "false") == "true";
    lm_config.forget_far_chunks = mod.arg("forget_far_chunks", "false") == "true";
    lm_config.visual_confidence_factor = mod.arg_float("visual_confidence_factor", 0.5);
    lm_config.pos_degeneracy_threshold = mod.arg_float("pos_degeneracy_threshold", 10.0);
    lm_config.ori_degeneracy_threshold = mod.arg_float("ori_degeneracy_threshold", 10.0);
    lm_config.shift_avg_ratio = mod.arg_float("shift_avg_ratio", 0.5);
    lm_config.shift_undistortion = mod.arg("shift_undistortion", "false") == "true";
    lm_config.yaw_ratio = mod.arg_float("yaw_ratio", 1.0);
    lm_config.relocalization_map_path = mod.arg("relocalization_map_path", "");
    lm_config.local_mode = mod.arg("local_mode", "false") == "true";
    lm_config.init_x = mod.arg_float("init_x", 0.0);
    lm_config.init_y = mod.arg_float("init_y", 0.0);
    lm_config.init_z = mod.arg_float("init_z", 0.0);
    lm_config.init_roll = mod.arg_float("init_roll", 0.0);
    lm_config.init_pitch = mod.arg_float("init_pitch", 0.0);
    lm_config.init_yaw = mod.arg_float("init_yaw", 0.0);
    lm_config.read_pose_file = mod.arg_float("read_pose_file", 0.0);

    // ---- IMU preintegration config ----
    g_has_imu_preintegration = mod.arg("use_imu_preintegration", "true") == "true";

    arise_slam::imuPreintegration_config imu_config;
    imu_config.imuAccNoise = mod.arg_float("acc_n", 0.01);
    imu_config.imuAccBiasN = mod.arg_float("acc_w", 0.0001);
    imu_config.imuGyrNoise = mod.arg_float("gyr_n", 0.001);
    imu_config.imuGyrBiasN = mod.arg_float("gyr_w", 0.00001);
    imu_config.imuGravity = mod.arg_float("g_norm", 9.81);
    imu_config.lidar_correction_noise = mod.arg_float("lidar_correction_noise", 0.05);
    imu_config.smooth_factor = mod.arg_float("smooth_factor", 0.1);
    imu_config.use_imu_roll_pitch = use_imu_roll_pitch;
    imu_config.lidar_flip = lidar_flip;
    imu_config.sensor = sensor_type;
    imu_config.imu_acc_x_offset = imu_acc_x_offset;
    imu_config.imu_acc_y_offset = imu_acc_y_offset;
    imu_config.imu_acc_z_offset = imu_acc_z_offset;
    imu_config.imu_acc_x_limit = imu_acc_x_limit;
    imu_config.imu_acc_y_limit = imu_acc_y_limit;
    imu_config.imu_acc_z_limit = imu_acc_z_limit;

    // ---- Set global calibration parameters (from parameter.h) ----
    // These globals are used internally by the arise_slam library
    sensor = sensor_type;
    readGlobalparam();
    readCalibration();

    // ---- Print config summary ----
    printf("[arise_slam] Starting LiDAR-Inertial SLAM module\n");
    printf("[arise_slam] lidar: %s | imu: %s\n",
           g_lidar_topic.c_str(), g_imu_topic.c_str());
    printf("[arise_slam] odom: %s | scan: %s\n",
           g_odom_topic.c_str(), g_scan_topic.c_str());
    printf("[arise_slam] sensor=%s scan_line=%d use_imu_roll_pitch=%s\n",
           sensor_str.c_str(), fe_config.N_SCANS,
           use_imu_roll_pitch ? "true" : "false");
    printf("[arise_slam] mapping: line_res=%.2f plane_res=%.2f max_iter=%d\n",
           lm_config.lineRes, lm_config.planeRes, lm_config.max_iterations);
    printf("[arise_slam] imu_preintegration=%s acc_n=%.4f gyr_n=%.5f\n",
           g_has_imu_preintegration ? "on" : "off",
           imu_config.imuAccNoise, imu_config.imuGyrNoise);

    // ---- Signal handlers ----
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    // ---- Init LCM ----
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[arise_slam] Error: LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    // ---- Initialize algorithm components ----
    printf("[arise_slam] Initializing feature extraction...\n");
    if (!g_feature_extraction.init(fe_config)) {
        fprintf(stderr, "[arise_slam] Error: feature extraction init failed\n");
        return 1;
    }

    printf("[arise_slam] Initializing laser mapping...\n");
    g_laser_mapping.init(lm_config);

    if (g_has_imu_preintegration) {
        printf("[arise_slam] Initializing IMU preintegration...\n");
        g_imu_preintegration.init(imu_config);
    }

    // ---- Set up output callbacks for IMU preintegration ----
    // The IMU preintegration publishes high-rate odometry between lidar frames
    if (g_has_imu_preintegration) {
        g_imu_preintegration.setOutputCallback([](const arise_slam::ImuState& state) {
            publish_odometry(state);
        });
    }

    // ---- Subscribe to LCM inputs ----
    lcm::LCM::HandlerFunction<sensor_msgs::PointCloud2> lidar_handler = on_lidar;
    lcm::LCM::HandlerFunction<sensor_msgs::Imu> imu_handler = on_imu;
    lcm.subscribe(g_lidar_topic, lidar_handler);
    lcm.subscribe(g_imu_topic, imu_handler);

    printf("[arise_slam] Initialized. Waiting for data...\n");

    // ---- Main loop ----
    while (g_running.load()) {
        lcm.handleTimeout(10);
    }

    // ---- Cleanup ----
    printf("[arise_slam] Shutting down...\n");
    g_lcm = nullptr;
    printf("[arise_slam] Done.\n");
    return 0;
}
