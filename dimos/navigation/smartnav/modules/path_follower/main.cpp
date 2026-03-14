// Path Follower - dimos NativeModule port of pathFollower.cpp
//
// Pure pursuit + PID yaw controller for path tracking.
// Subscribes to path and odometry over LCM, publishes cmd_vel (Twist).
//
// Original: src/base_autonomy/local_planner/src/pathFollower.cpp

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <mutex>

#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

#include "nav_msgs/Odometry.hpp"
#include "nav_msgs/Path.hpp"
#include "geometry_msgs/Twist.hpp"

using namespace std;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static const double PI = 3.1415926;

static double normalizeAngle(double angle) {
    return atan2(sin(angle), cos(angle));
}

// ---------------------------------------------------------------------------
// Wall-clock helper (replaces rclcpp::Time / node->now())
// ---------------------------------------------------------------------------
static double now_seconds() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(
        steady_clock::now().time_since_epoch()).count();
}

static double stamp_to_seconds(const std_msgs::Time& t) {
    return t.sec + t.nsec / 1.0e9;
}

// ---------------------------------------------------------------------------
// Tuneable parameters (loaded from CLI args via NativeModule)
// ---------------------------------------------------------------------------
static double sensorOffsetX = 0;
static double sensorOffsetY = 0;
static int    pubSkipNum = 1;
static int    pubSkipCount = 0;
static bool   twoWayDrive = true;
static double lookAheadDis = 0.5;
static double yawRateGain = 7.5;
static double stopYawRateGain = 7.5;
static double maxYawRate = 45.0;
static double maxSpeed = 1.0;
static double maxAccel = 1.0;
static double switchTimeThre = 1.0;
static double dirDiffThre = 0.1;
static double omniDirGoalThre = 1.0;
static double omniDirDiffThre = 1.5;
static double stopDisThre = 0.2;
static double slowDwnDisThre = 1.0;
static bool   useInclRateToSlow = false;
static double inclRateThre = 120.0;
static double slowRate1 = 0.25;
static double slowRate2 = 0.5;
static double slowRate3 = 0.75;
static double slowTime1 = 2.0;
static double slowTime2 = 2.0;
static bool   useInclToStop = false;
static double inclThre = 45.0;
static double stopTime = 5.0;
static bool   noRotAtStop = false;
static bool   noRotAtGoal = true;
static bool   autonomyMode = false;
static double autonomySpeed = 1.0;
static double goalYawGain = 2.0;

// ---------------------------------------------------------------------------
// Runtime state (mirrors the original globals)
// ---------------------------------------------------------------------------
static double goalYaw = 0;
static bool   hasGoalYaw = false;

static float joySpeed = 0;
static float joyYaw = 0;

static float vehicleX = 0;
static float vehicleY = 0;
static float vehicleZ = 0;
static float vehicleRoll = 0;
static float vehiclePitch = 0;
static float vehicleYaw = 0;

static float vehicleXRec = 0;
static float vehicleYRec = 0;
static float vehicleZRec = 0;
static float vehicleRollRec = 0;
static float vehiclePitchRec = 0;
static float vehicleYawRec = 0;

static float vehicleYawRate = 0;
static float vehicleSpeed = 0;

static double odomTime = 0;
static double slowInitTime = 0;
static double stopInitTime = 0;
static int    pathPointID = 0;
static bool   pathInit = false;
static bool   navFwd = true;
static double switchTime = 0;

static int    safetyStop = 0;
static int    slowDown = 0;

// Path storage  (we keep a simple vector of poses)
struct SimplePose {
    double x, y, z;
    double qx, qy, qz, qw;
};
static std::vector<SimplePose> pathPoses;
static std::mutex pathMutex;

// ---------------------------------------------------------------------------
// LCM Callbacks
// ---------------------------------------------------------------------------
class Handlers {
public:
    // Odometry handler -------------------------------------------------------
    void odomHandler(const lcm::ReceiveBuffer* /*rbuf*/,
                     const std::string& /*channel*/,
                     const nav_msgs::Odometry* msg)
    {
        odomTime = stamp_to_seconds(msg->header.stamp);

        double roll, pitch, yaw;
        const auto& q = msg->pose.pose.orientation;
        smartnav::quat_to_rpy(q.x, q.y, q.z, q.w, roll, pitch, yaw);

        vehicleRoll  = static_cast<float>(roll);
        vehiclePitch = static_cast<float>(pitch);
        vehicleYaw   = static_cast<float>(yaw);
        vehicleX = static_cast<float>(msg->pose.pose.position.x
                                      - cos(yaw) * sensorOffsetX
                                      + sin(yaw) * sensorOffsetY);
        vehicleY = static_cast<float>(msg->pose.pose.position.y
                                      - sin(yaw) * sensorOffsetX
                                      - cos(yaw) * sensorOffsetY);
        vehicleZ = static_cast<float>(msg->pose.pose.position.z);

        if ((fabs(roll) > inclThre * PI / 180.0 ||
             fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
            stopInitTime = stamp_to_seconds(msg->header.stamp);
        }

        if ((fabs(msg->twist.twist.angular.x) > inclRateThre * PI / 180.0 ||
             fabs(msg->twist.twist.angular.y) > inclRateThre * PI / 180.0) &&
            useInclRateToSlow) {
            slowInitTime = stamp_to_seconds(msg->header.stamp);
        }
    }

    // Path handler -----------------------------------------------------------
    void pathHandler(const lcm::ReceiveBuffer* /*rbuf*/,
                     const std::string& /*channel*/,
                     const nav_msgs::Path* msg)
    {
        std::lock_guard<std::mutex> lock(pathMutex);

        int pathSize = msg->poses_length;
        pathPoses.resize(pathSize);
        for (int i = 0; i < pathSize; i++) {
            pathPoses[i].x  = msg->poses[i].pose.position.x;
            pathPoses[i].y  = msg->poses[i].pose.position.y;
            pathPoses[i].z  = msg->poses[i].pose.position.z;
            pathPoses[i].qx = msg->poses[i].pose.orientation.x;
            pathPoses[i].qy = msg->poses[i].pose.orientation.y;
            pathPoses[i].qz = msg->poses[i].pose.orientation.z;
            pathPoses[i].qw = msg->poses[i].pose.orientation.w;
        }

        if (pathSize > 0) {
            const auto& lo = pathPoses[pathSize - 1];
            if (lo.qw != 0 || lo.qx != 0 || lo.qy != 0 || lo.qz != 0) {
                double roll, pitch, yaw;
                smartnav::quat_to_rpy(lo.qx, lo.qy, lo.qz, lo.qw,
                                      roll, pitch, yaw);
                goalYaw = yaw;
                hasGoalYaw = true;
            } else {
                hasGoalYaw = false;
            }
        } else {
            hasGoalYaw = false;
        }

        vehicleXRec     = vehicleX;
        vehicleYRec     = vehicleY;
        vehicleZRec     = vehicleZ;
        vehicleRollRec  = vehicleRoll;
        vehiclePitchRec = vehiclePitch;
        vehicleYawRec   = vehicleYaw;

        pathPointID = 0;
        pathInit = true;
    }
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // --- Parse CLI args via NativeModule ---
    dimos::NativeModule mod(argc, argv);

    sensorOffsetX    = mod.arg_float("sensorOffsetX",    static_cast<float>(sensorOffsetX));
    sensorOffsetY    = mod.arg_float("sensorOffsetY",    static_cast<float>(sensorOffsetY));
    pubSkipNum       = mod.arg_int  ("pubSkipNum",       pubSkipNum);
    twoWayDrive      = mod.arg_bool ("twoWayDrive",      twoWayDrive);
    lookAheadDis     = mod.arg_float("lookAheadDis",     static_cast<float>(lookAheadDis));
    yawRateGain      = mod.arg_float("yawRateGain",      static_cast<float>(yawRateGain));
    stopYawRateGain  = mod.arg_float("stopYawRateGain",  static_cast<float>(stopYawRateGain));
    maxYawRate       = mod.arg_float("maxYawRate",        static_cast<float>(maxYawRate));
    maxSpeed         = mod.arg_float("maxSpeed",          static_cast<float>(maxSpeed));
    maxAccel         = mod.arg_float("maxAccel",          static_cast<float>(maxAccel));
    switchTimeThre   = mod.arg_float("switchTimeThre",    static_cast<float>(switchTimeThre));
    dirDiffThre      = mod.arg_float("dirDiffThre",       static_cast<float>(dirDiffThre));
    omniDirGoalThre  = mod.arg_float("omniDirGoalThre",  static_cast<float>(omniDirGoalThre));
    omniDirDiffThre  = mod.arg_float("omniDirDiffThre",  static_cast<float>(omniDirDiffThre));
    stopDisThre      = mod.arg_float("stopDisThre",       static_cast<float>(stopDisThre));
    slowDwnDisThre   = mod.arg_float("slowDwnDisThre",    static_cast<float>(slowDwnDisThre));
    useInclRateToSlow= mod.arg_bool ("useInclRateToSlow", useInclRateToSlow);
    inclRateThre     = mod.arg_float("inclRateThre",      static_cast<float>(inclRateThre));
    slowRate1        = mod.arg_float("slowRate1",         static_cast<float>(slowRate1));
    slowRate2        = mod.arg_float("slowRate2",         static_cast<float>(slowRate2));
    slowRate3        = mod.arg_float("slowRate3",         static_cast<float>(slowRate3));
    slowTime1        = mod.arg_float("slowTime1",         static_cast<float>(slowTime1));
    slowTime2        = mod.arg_float("slowTime2",         static_cast<float>(slowTime2));
    useInclToStop    = mod.arg_bool ("useInclToStop",     useInclToStop);
    inclThre         = mod.arg_float("inclThre",          static_cast<float>(inclThre));
    stopTime         = mod.arg_float("stopTime",          static_cast<float>(stopTime));
    noRotAtStop      = mod.arg_bool ("noRotAtStop",       noRotAtStop);
    noRotAtGoal      = mod.arg_bool ("noRotAtGoal",       noRotAtGoal);
    autonomyMode     = mod.arg_bool ("autonomyMode",      autonomyMode);
    autonomySpeed    = mod.arg_float("autonomySpeed",     static_cast<float>(autonomySpeed));
    goalYawGain      = mod.arg_float("goalYawGain",       static_cast<float>(goalYawGain));

    // --- Resolve LCM topics ---
    const std::string pathTopic = mod.topic("path");
    const std::string odomTopic = mod.topic("odometry");
    const std::string cmdTopic  = mod.topic("cmd_vel");

    // --- Create LCM instance ---
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[path_follower] ERROR: LCM init failed\n");
        return 1;
    }

    // --- Subscribe ---
    Handlers handlers;
    lcm.subscribe(odomTopic, &Handlers::odomHandler, &handlers);
    lcm.subscribe(pathTopic, &Handlers::pathHandler,  &handlers);

    // --- Initial speed for autonomy mode ---
    if (autonomyMode) {
        joySpeed = static_cast<float>(autonomySpeed / maxSpeed);
        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0f) joySpeed = 1.0f;
    }

    printf("[path_follower] Running.  path=%s  odom=%s  cmd=%s\n",
            pathTopic.c_str(), odomTopic.c_str(), cmdTopic.c_str());
    fflush(stdout);

    // --- Main loop at 100 Hz ---
    const auto loopPeriod = std::chrono::milliseconds(10);

    while (true) {
        // Non-blocking drain of all pending LCM messages
        while (lcm.handleTimeout(0) > 0) {}

        if (pathInit) {
            std::lock_guard<std::mutex> lock(pathMutex);

            float vehicleXRel =  cos(vehicleYawRec) * (vehicleX - vehicleXRec)
                               + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
            float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec)
                               + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

            int pathSize = static_cast<int>(pathPoses.size());
            if (pathSize <= 0) { pathInit = false; continue; }
            float endDisX = static_cast<float>(pathPoses[pathSize - 1].x) - vehicleXRel;
            float endDisY = static_cast<float>(pathPoses[pathSize - 1].y) - vehicleYRel;
            float endDis  = sqrt(endDisX * endDisX + endDisY * endDisY);

            // Advance along path until look-ahead distance is reached
            float disX, disY, dis;
            while (pathPointID < pathSize - 1) {
                disX = static_cast<float>(pathPoses[pathPointID].x) - vehicleXRel;
                disY = static_cast<float>(pathPoses[pathPointID].y) - vehicleYRel;
                dis  = sqrt(disX * disX + disY * disY);
                if (dis < lookAheadDis) {
                    pathPointID++;
                } else {
                    break;
                }
            }

            disX = static_cast<float>(pathPoses[pathPointID].x) - vehicleXRel;
            disY = static_cast<float>(pathPoses[pathPointID].y) - vehicleYRel;
            dis  = sqrt(disX * disX + disY * disY);
            float pathDir = atan2(disY, disX);

            // Direction difference (vehicle heading vs path direction)
            float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
            if (dirDiff >  PI) dirDiff -= 2 * PI;
            else if (dirDiff < -PI) dirDiff += 2 * PI;
            if (dirDiff >  PI) dirDiff -= 2 * PI;
            else if (dirDiff < -PI) dirDiff += 2 * PI;

            // Two-way drive: switch forward/reverse
            if (twoWayDrive) {
                double time = now_seconds();
                if (fabs(dirDiff) > PI / 2 && navFwd &&
                    time - switchTime > switchTimeThre) {
                    navFwd = false;
                    switchTime = time;
                } else if (fabs(dirDiff) < PI / 2 && !navFwd &&
                           time - switchTime > switchTimeThre) {
                    navFwd = true;
                    switchTime = time;
                }
            }

            float joySpeed2 = static_cast<float>(maxSpeed) * joySpeed;
            if (!navFwd) {
                dirDiff += static_cast<float>(PI);
                if (dirDiff > PI) dirDiff -= 2 * PI;
                joySpeed2 *= -1;
            }

            // PID yaw controller
            if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0)
                vehicleYawRate = static_cast<float>(-stopYawRateGain * dirDiff);
            else
                vehicleYawRate = static_cast<float>(-yawRateGain * dirDiff);

            if (vehicleYawRate >  maxYawRate * PI / 180.0)
                vehicleYawRate = static_cast<float>(maxYawRate * PI / 180.0);
            else if (vehicleYawRate < -maxYawRate * PI / 180.0)
                vehicleYawRate = static_cast<float>(-maxYawRate * PI / 180.0);

            // Goal yaw alignment when near the end of the path
            if (hasGoalYaw && pathPointID >= pathSize - 1 &&
                endDis < stopDisThre && !noRotAtGoal) {
                double yawError = normalizeAngle(goalYaw - vehicleYaw);
                vehicleYawRate = static_cast<float>(goalYawGain * yawError);
                if (vehicleYawRate >  maxYawRate * PI / 180.0)
                    vehicleYawRate = static_cast<float>(maxYawRate * PI / 180.0);
                else if (vehicleYawRate < -maxYawRate * PI / 180.0)
                    vehicleYawRate = static_cast<float>(-maxYawRate * PI / 180.0);
                joySpeed2 = 0;
            }

            // Yaw behaviour when stopped / at goal
            if (joySpeed2 == 0 && !autonomyMode) {
                vehicleYawRate = static_cast<float>(maxYawRate * joyYaw * PI / 180.0);
            } else if ((pathSize <= 1 && !hasGoalYaw) ||
                       (dis < stopDisThre && noRotAtGoal && !hasGoalYaw)) {
                vehicleYawRate = 0;
            }

            // Speed limiting near end of path
            if (pathSize <= 1) {
                joySpeed2 = 0;
            } else if (endDis / slowDwnDisThre < joySpeed) {
                joySpeed2 *= endDis / static_cast<float>(slowDwnDisThre);
            }

            // Inclination / slow-down rate adjustments
            float joySpeed3 = joySpeed2;
            if ((odomTime < slowInitTime + slowTime1 && slowInitTime > 0) ||
                slowDown == 1)
                joySpeed3 *= static_cast<float>(slowRate1);
            else if ((odomTime < slowInitTime + slowTime1 + slowTime2 &&
                      slowInitTime > 0) || slowDown == 2)
                joySpeed3 *= static_cast<float>(slowRate2);
            else if (slowDown == 3)
                joySpeed3 *= static_cast<float>(slowRate3);

            // Acceleration / deceleration ramp
            if ((fabs(dirDiff) < dirDiffThre ||
                 (dis < omniDirGoalThre && fabs(dirDiff) < omniDirDiffThre)) &&
                dis > stopDisThre) {
                if (vehicleSpeed < joySpeed3)
                    vehicleSpeed += static_cast<float>(maxAccel / 100.0);
                else if (vehicleSpeed > joySpeed3)
                    vehicleSpeed -= static_cast<float>(maxAccel / 100.0);
            } else {
                if (vehicleSpeed > 0)
                    vehicleSpeed -= static_cast<float>(maxAccel / 100.0);
                else if (vehicleSpeed < 0)
                    vehicleSpeed += static_cast<float>(maxAccel / 100.0);
            }

            // Inclination stop
            if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
                vehicleSpeed = 0;
                vehicleYawRate = 0;
            }

            // Safety stop
            if (safetyStop >= 1) vehicleSpeed = 0;
            if (safetyStop >= 2) vehicleYawRate = 0;

            // --- Publish cmd_vel ---
            pubSkipCount--;
            if (pubSkipCount < 0) {
                geometry_msgs::Twist cmd_vel;

                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.linear.z = 0;
                cmd_vel.angular.x = 0;
                cmd_vel.angular.y = 0;
                cmd_vel.angular.z = vehicleYawRate;

                if (fabs(vehicleSpeed) > maxAccel / 100.0) {
                    if (omniDirGoalThre > 0) {
                        cmd_vel.linear.x =  cos(dirDiff) * vehicleSpeed;
                        cmd_vel.linear.y = -sin(dirDiff) * vehicleSpeed;
                    } else {
                        cmd_vel.linear.x = vehicleSpeed;
                    }
                }

                lcm.publish(cmdTopic, &cmd_vel);
                pubSkipCount = pubSkipNum;
            }
        }

        std::this_thread::sleep_for(loopPeriod);
    }

    return 0;
}
