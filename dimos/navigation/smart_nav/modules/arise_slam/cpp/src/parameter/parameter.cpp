//
// Created by shibo zhao on 2020-09-27.
//
#include "arise_slam_mid360/config/parameter.h"

// Define color escape codes for ~beautification~
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

#define BOLD "\033[1m"
#define UNDERLINE "\033[4m"
#define ITALIC "\033[3m"

std::string IMU_TOPIC;
std::string LASER_TOPIC;
std::string ODOM_TOPIC;
std::string DepthUP_TOPIC;
std::string DepthDown_TOPIC;
std::string ProjectName;

std::string WORLD_FRAME;
std::string WORLD_FRAME_ROT;
std::string SENSOR_FRAME;
std::string SENSOR_FRAME_ROT;
SensorType sensor;

int PROVIDE_IMU_LASER_EXTRINSIC;

Eigen::Matrix3d imu_laser_R;

Eigen::Vector3d imu_laser_T;

Eigen::Vector3d imu_laser_offset;

Eigen::Matrix3d cam_laser_R;

Eigen::Vector3d cam_laser_T;

Eigen::Matrix3d imu_camera_R;

Eigen::Vector3d imu_camera_T;

Transformd Tcam_lidar;

Transformd T_i_c;

Transformd T_i_l;

Transformd T_l_i;



float lidar_imu_offset_roll;



float up_realsense_roll;

float up_realsense_pitch;

float up_realsense_yaw;

float up_realsense_x;

float up_realsense_y;

float up_realsense_z;

float down_realsense_roll;

float down_realsense_pitch;

float down_realsense_yaw;

float down_realsense_x;

float down_realsense_y;

float down_realsense_z;

float yaw_ratio;



// float min_range;



// float max_range;

// float blindFront;

// float blindBack;

// float blindRight;

// float blindLeft;

// int skipFrame;

// int N_SCANS;

// float scan_registration_voxel_size;

//float lidar_correction_noise;

// bool use_no_motion_prior;

//float smooth_factor;

//bool use_imu_roll_pitch;

// bool start_from_previous_map;

// main.cpp will set calibration globals directly -- this is now a no-op
bool readCalibration()
{
    // Calibration parameters are set by main.cpp from CLI args / config file
    return true;
}

// main.cpp will set the global topic/frame variables directly -- this is now a no-op
bool readGlobalparam()
{
    // Global parameters are set by main.cpp from CLI args
    return true;
}
