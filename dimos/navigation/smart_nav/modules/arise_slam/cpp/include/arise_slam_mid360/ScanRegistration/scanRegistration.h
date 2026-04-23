//
// Created by shibo zhao on 2020-09-27.
// Ported from ROS2 to plain C++ for DimOS (LCM-based)
//
// Note: Scan registration logic is handled by featureExtraction module.
// This header is kept as a minimal stub for compatibility.
//

#ifndef arise_slam_mid360_SCANREGISTRATION_H
#define arise_slam_mid360_SCANREGISTRATION_H

#include <cmath>
#include <string>
#include <vector>
#include <mutex>
#include <iomanip>

#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "arise_slam_mid360/common.h"
#include "arise_slam_mid360/config/parameter.h"
#include "arise_slam_mid360/container/MapRingBuffer.h"
#include "arise_slam_mid360/sensor_data/imu/imu_data.h"
#include "arise_slam_mid360/sensor_data/pointcloud/point_os.h"
#include "arise_slam_mid360/tic_toc.h"
#include "arise_slam_mid360/utils/Twist.h"

namespace arise_slam
{

    struct bounds_t
    {
        double blindFront;
        double blindBack;
        double blindRight;
        double blindLeft;
        double blindUp;
        double blindDown;
    };

    struct scan_registration_config{
        bounds_t box_size;
        int skipFrame;
        int N_SCANS;
        std::string IMU_TOPIC;
        std::string LASER_TOPIC;
        std::string ODOM_TOPIC;
        float min_range;
        float max_range;
        int downsampleRate;
        int skip_point;
        SensorType sensor;
    };

    float cloudCurvature[400000];
    int cloudSortInd[400000];
    int cloudNeighborPicked[400000];
    int cloudLabel[400000];

    bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

} // namespace arise_slam

#endif //arise_slam_mid360_SCANREGISTRATION_H
