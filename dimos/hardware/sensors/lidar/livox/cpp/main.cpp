// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Livox Mid-360 native module for dimos NativeModule framework.
//
// Publishes PointCloud2 and Imu messages on LCM topics received via CLI args.
// Usage: ./mid360_native --pointcloud <topic> --imu <topic> [--host_ip <ip>] [--lidar_ip <ip>] ...

#include <lcm/lcm-cpp.hpp>
#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dimos_native_module.hpp"

#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Header.hpp"
#include "std_msgs/Time.hpp"

// Gravity constant for converting accelerometer data from g to m/s^2
static constexpr double GRAVITY_MS2 = 9.80665;

// Livox data_type values (not provided as named constants in SDK2 header)
static constexpr uint8_t DATA_TYPE_IMU = 0x00;
static constexpr uint8_t DATA_TYPE_CARTESIAN_HIGH = 0x01;
static constexpr uint8_t DATA_TYPE_CARTESIAN_LOW = 0x02;

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;
static std::string g_pointcloud_topic;
static std::string g_imu_topic;
static std::string g_frame_id = "lidar_link";
static std::string g_imu_frame_id = "imu_link";
static float g_frequency = 10.0f;

// Frame accumulator
static std::mutex g_pc_mutex;
static std::vector<float> g_accumulated_xyz;       // interleaved x,y,z
static std::vector<float> g_accumulated_intensity;  // per-point intensity
static double g_frame_timestamp = 0.0;
static bool g_frame_has_timestamp = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std_msgs::Time time_from_seconds(double t) {
    std_msgs::Time ts;
    ts.sec = static_cast<int32_t>(t);
    ts.nsec = static_cast<int32_t>((t - ts.sec) * 1e9);
    return ts;
}

static double get_timestamp_ns(const LivoxLidarEthernetPacket* pkt) {
    uint64_t ns = 0;
    std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
    return static_cast<double>(ns);
}

static std_msgs::Header make_header(const std::string& frame_id, double ts) {
    static std::atomic<int32_t> seq{0};
    std_msgs::Header h;
    h.seq = seq.fetch_add(1, std::memory_order_relaxed);
    h.stamp = time_from_seconds(ts);
    h.frame_id = frame_id;
    return h;
}

// ---------------------------------------------------------------------------
// Build and publish PointCloud2
// ---------------------------------------------------------------------------

static void publish_pointcloud(const std::vector<float>& xyz,
                               const std::vector<float>& intensity,
                               double timestamp) {
    if (!g_lcm || xyz.empty()) return;

    int num_points = static_cast<int>(xyz.size()) / 3;

    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(g_frame_id, timestamp);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    // Fields: x, y, z (float32), intensity (float32)
    pc.fields_length = 4;
    pc.fields.resize(4);

    auto make_field = [](const std::string& name, int32_t offset) {
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

    pc.point_step = 16;  // 4 floats * 4 bytes
    pc.row_step = pc.point_step * num_points;

    // Pack point data
    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);

    for (int i = 0; i < num_points; ++i) {
        float* dst = reinterpret_cast<float*>(pc.data.data() + i * 16);
        dst[0] = xyz[i * 3 + 0];
        dst[1] = xyz[i * 3 + 1];
        dst[2] = xyz[i * 3 + 2];
        dst[3] = intensity[i];
    }

    g_lcm->publish(g_pointcloud_topic, &pc);
}

// ---------------------------------------------------------------------------
// SDK callbacks
// ---------------------------------------------------------------------------

static void on_point_cloud(const uint32_t /*handle*/, const uint8_t /*dev_type*/,
                           LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr) return;

    double ts_ns = get_timestamp_ns(data);
    double ts = ts_ns / 1e9;
    uint16_t dot_num = data->dot_num;

    std::lock_guard<std::mutex> lock(g_pc_mutex);

    if (!g_frame_has_timestamp) {
        g_frame_timestamp = ts;
        g_frame_has_timestamp = true;
    }

    if (data->data_type == DATA_TYPE_CARTESIAN_HIGH) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
        for (uint16_t i = 0; i < dot_num; ++i) {
            // Livox high-precision coordinates are in mm, convert to meters
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].x) / 1000.0f);
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].y) / 1000.0f);
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].z) / 1000.0f);
            g_accumulated_intensity.push_back(static_cast<float>(pts[i].reflectivity) / 255.0f);
        }
    } else if (data->data_type == DATA_TYPE_CARTESIAN_LOW) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
        for (uint16_t i = 0; i < dot_num; ++i) {
            // Livox low-precision coordinates are in cm, convert to meters
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].x) / 100.0f);
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].y) / 100.0f);
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].z) / 100.0f);
            g_accumulated_intensity.push_back(static_cast<float>(pts[i].reflectivity) / 255.0f);
        }
    }
}

static void on_imu_data(const uint32_t /*handle*/, const uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr || !g_lcm) return;
    if (g_imu_topic.empty()) return;

    double ts = get_timestamp_ns(data) / 1e9;
    auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
    uint16_t dot_num = data->dot_num;

    for (uint16_t i = 0; i < dot_num; ++i) {
        sensor_msgs::Imu msg;
        msg.header = make_header(g_imu_frame_id, ts);

        // Orientation unknown — set to identity with high covariance
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        msg.orientation_covariance[0] = -1.0;  // indicates unknown

        msg.angular_velocity.x = static_cast<double>(imu_pts[i].gyro_x);
        msg.angular_velocity.y = static_cast<double>(imu_pts[i].gyro_y);
        msg.angular_velocity.z = static_cast<double>(imu_pts[i].gyro_z);

        msg.linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x) * GRAVITY_MS2;
        msg.linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y) * GRAVITY_MS2;
        msg.linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z) * GRAVITY_MS2;

        g_lcm->publish(g_imu_topic, &msg);
    }
}

static void on_info_change(const uint32_t handle, const LivoxLidarInfo* info,
                           void* /*client_data*/) {
    if (info == nullptr) return;

    char sn[17] = {};
    std::memcpy(sn, info->sn, 16);
    char ip[17] = {};
    std::memcpy(ip, info->lidar_ip, 16);

    printf("[mid360] Device connected: handle=%u type=%u sn=%s ip=%s\n",
           handle, info->dev_type, sn, ip);

    // Set to normal work mode
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);

    // Enable IMU
    if (!g_imu_topic.empty()) {
        EnableLivoxLidarImuData(handle, nullptr, nullptr);
    }
}

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

// ---------------------------------------------------------------------------
// SDK config file generation
// ---------------------------------------------------------------------------

struct SdkPorts {
    int cmd_data      = 56100;
    int push_msg      = 56200;
    int point_data    = 56300;
    int imu_data      = 56400;
    int log_data      = 56500;
    int host_cmd_data   = 56101;
    int host_push_msg   = 56201;
    int host_point_data = 56301;
    int host_imu_data   = 56401;
    int host_log_data   = 56501;
};

static std::string write_sdk_config(const std::string& host_ip,
                                    const std::string& lidar_ip,
                                    const SdkPorts& ports) {
    char tmpl[] = "/tmp/livox_mid360_XXXXXX";
    int fd = mkstemp(tmpl);
    if (fd < 0) {
        perror("mkstemp");
        return "";
    }

    // fdopen takes ownership of fd — fclose will close it
    FILE* fp = fdopen(fd, "w");
    if (!fp) {
        perror("fdopen");
        close(fd);
        return "";
    }

    fprintf(fp,
        "{\n"
        "  \"MID360\": {\n"
        "    \"lidar_net_info\": {\n"
        "      \"cmd_data_port\": %d,\n"
        "      \"push_msg_port\": %d,\n"
        "      \"point_data_port\": %d,\n"
        "      \"imu_data_port\": %d,\n"
        "      \"log_data_port\": %d\n"
        "    },\n"
        "    \"host_net_info\": [\n"
        "      {\n"
        "        \"host_ip\": \"%s\",\n"
        "        \"multicast_ip\": \"224.1.1.5\",\n"
        "        \"cmd_data_port\": %d,\n"
        "        \"push_msg_port\": %d,\n"
        "        \"point_data_port\": %d,\n"
        "        \"imu_data_port\": %d,\n"
        "        \"log_data_port\": %d\n"
        "      }\n"
        "    ]\n"
        "  }\n"
        "}\n",
        ports.cmd_data, ports.push_msg, ports.point_data,
        ports.imu_data, ports.log_data,
        host_ip.c_str(),
        ports.host_cmd_data, ports.host_push_msg, ports.host_point_data,
        ports.host_imu_data, ports.host_log_data);
    fclose(fp);

    return tmpl;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // Required: LCM topics for ports
    g_pointcloud_topic = mod.has("pointcloud") ? mod.topic("pointcloud") : "";
    g_imu_topic = mod.has("imu") ? mod.topic("imu") : "";

    if (g_pointcloud_topic.empty()) {
        fprintf(stderr, "Error: --pointcloud <topic> is required\n");
        return 1;
    }

    // Optional config args
    std::string host_ip = mod.arg("host_ip", "192.168.1.5");
    std::string lidar_ip = mod.arg("lidar_ip", "192.168.1.155");
    g_frequency = mod.arg_float("frequency", 10.0f);
    g_frame_id = mod.arg("frame_id", "lidar_link");
    g_imu_frame_id = mod.arg("imu_frame_id", "imu_link");

    // SDK network ports (configurable for multi-sensor setups)
    SdkPorts ports;
    ports.cmd_data        = mod.arg_int("cmd_data_port", 56100);
    ports.push_msg        = mod.arg_int("push_msg_port", 56200);
    ports.point_data      = mod.arg_int("point_data_port", 56300);
    ports.imu_data        = mod.arg_int("imu_data_port", 56400);
    ports.log_data        = mod.arg_int("log_data_port", 56500);
    ports.host_cmd_data   = mod.arg_int("host_cmd_data_port", 56101);
    ports.host_push_msg   = mod.arg_int("host_push_msg_port", 56201);
    ports.host_point_data = mod.arg_int("host_point_data_port", 56301);
    ports.host_imu_data   = mod.arg_int("host_imu_data_port", 56401);
    ports.host_log_data   = mod.arg_int("host_log_data_port", 56501);

    printf("[mid360] Starting native Livox Mid-360 module\n");
    printf("[mid360] pointcloud topic: %s\n", g_pointcloud_topic.c_str());
    printf("[mid360] imu topic: %s\n", g_imu_topic.empty() ? "(disabled)" : g_imu_topic.c_str());
    printf("[mid360] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n",
           host_ip.c_str(), lidar_ip.c_str(), g_frequency);

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

    // Write SDK config
    std::string config_path = write_sdk_config(host_ip, lidar_ip, ports);
    if (config_path.empty()) {
        fprintf(stderr, "Error: failed to write SDK config\n");
        return 1;
    }

    // Init Livox SDK
    if (!LivoxLidarSdkInit(config_path.c_str(), host_ip.c_str())) {
        fprintf(stderr, "Error: LivoxLidarSdkInit failed\n");
        std::remove(config_path.c_str());
        return 1;
    }

    // Register callbacks
    SetLivoxLidarPointCloudCallBack(on_point_cloud, nullptr);
    if (!g_imu_topic.empty()) {
        SetLivoxLidarImuDataCallback(on_imu_data, nullptr);
    }
    SetLivoxLidarInfoChangeCallback(on_info_change, nullptr);

    // Start SDK
    if (!LivoxLidarSdkStart()) {
        fprintf(stderr, "Error: LivoxLidarSdkStart failed\n");
        LivoxLidarSdkUninit();
        std::remove(config_path.c_str());
        return 1;
    }

    printf("[mid360] SDK started, waiting for device...\n");

    // Main loop: periodically emit accumulated point clouds
    auto frame_interval = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / g_frequency));
    auto last_emit = std::chrono::steady_clock::now();

    while (g_running.load()) {
        // Handle LCM (for any subscriptions, though we mostly publish)
        lcm.handleTimeout(10);  // 10ms timeout

        auto now = std::chrono::steady_clock::now();
        if (now - last_emit >= frame_interval) {
            // Swap out the accumulated data
            std::vector<float> xyz;
            std::vector<float> intensity;
            double ts = 0.0;

            {
                std::lock_guard<std::mutex> lock(g_pc_mutex);
                if (!g_accumulated_xyz.empty()) {
                    xyz.swap(g_accumulated_xyz);
                    intensity.swap(g_accumulated_intensity);
                    ts = g_frame_timestamp;
                    g_frame_has_timestamp = false;
                }
            }

            if (!xyz.empty()) {
                publish_pointcloud(xyz, intensity, ts);
            }

            last_emit = now;
        }
    }

    // Cleanup
    printf("[mid360] Shutting down...\n");
    LivoxLidarSdkUninit();
    std::remove(config_path.c_str());
    g_lcm = nullptr;

    printf("[mid360] Done.\n");
    return 0;
}
