//
// Created by shibo zhao on 2020-09-27.
// Ported from ROS2 to plain C++ for DimOS (LCM-based)
//
#include "arise_slam_mid360/ImuPreintegration/imuPreintegration.h"

// For TransformFusion
static double lidarOdomTime2=-1;
static Eigen::Affine3f lidarOdomAffine;
namespace arise_slam {

    imuPreintegration::imuPreintegration() {
    }

    void imuPreintegration::init(const imuPreintegration_config& config) {
        config_ = config;

        // Allocate buffers
        imuBuf.allocate(1000);
        lidarOdomBuf.allocate(100);
        visualOdomBuf.allocate(5000);

        printf("[arise_slam] imuPreintegration: use_imu_roll_pitch: %d\n", config_.use_imu_roll_pitch);
        printf("[arise_slam] imuPreintegration: lidar_flip: %d\n", config_.lidar_flip);
        printf("[arise_slam] imuPreintegration: imuAccNoise: %f  imuAccBiasN: %f imuGyrNoise %f imuGyrBiasN %f imuGravity %f\n",
                    config_.imuAccNoise, config_.imuAccBiasN, config_.imuGyrNoise, config_.imuGyrBiasN, config_.imuGravity);
        printf("[arise_slam] imuPreintegration: imu_acc_x_limit: %f\n", config_.imu_acc_x_limit);
        printf("[arise_slam] imuPreintegration: imu_acc_y_limit: %f\n", config_.imu_acc_y_limit);
        printf("[arise_slam] imuPreintegration: imu_acc_z_limit: %f\n", config_.imu_acc_z_limit);

        // set relevant parameter
        std::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(config_.imuGravity);

        p->accelerometerCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuAccNoise, 2);
        p->gyroscopeCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuGyrNoise, 2);
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) *
                                   pow(1e-4, 2);

        gtsam::imuBias::ConstantBias prior_imu_bias(
                (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
        priorVisualPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-1);
        correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, config_.lidar_correction_noise);

        noMotionNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
        noVelocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);

        noiseModelBetweenBias = (gtsam::Vector(6)
                << config_.imuAccBiasN,
                config_.imuAccBiasN, config_.imuAccBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN)
                .finished();
        imuIntegratorImu_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);
        imuIntegratorOpt_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);

        //set extrinsic matrix for laser and imu
        if (PROVIDE_IMU_LASER_EXTRINSIC) {
            imu2Lidar = gtsam::Pose3(gtsam::Rot3(imu_laser_R), gtsam::Point3(imu_laser_T));
            lidar2Imu = imu2Lidar.inverse();
        } else {
            imu2cam = gtsam::Pose3(gtsam::Rot3(imu_camera_R), gtsam::Point3(imu_camera_T));
            cam2Lidar = gtsam::Pose3(gtsam::Rot3(cam_laser_R), gtsam::Point3(cam_laser_T));
            imu2Lidar = imu2cam.compose(cam2Lidar);
            lidar2Imu = imu2Lidar.inverse();
        }

        printf("[arise_slam] imuPreintegration: Initialized successfully\n");
    }

    void imuPreintegration::resetOptimization() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void imuPreintegration::resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void imuPreintegration::addNoMotionFactor(const FrameId &from_id, const FrameId &to_id)
    {

        Eigen::Matrix3d relative_motion;
        Eigen::Vector3d relative_trans;

        graphFactors.push_back(
                std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                        gtsam::Symbol('x', from_id),
                        gtsam::Symbol('x', to_id),
                        gtsam::Pose3(gtsam::Rot3(relative_motion.setIdentity()),
                                     gtsam::Point3(relative_trans.setZero())),
                        noMotionNoise));
    }

    void imuPreintegration::addZeroVelocityPrior(const FrameId &frame_id) {
        graphFactors.push_back(
                std::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
                        gtsam::Symbol('v', frame_id),
                        gtsam::Vector3::Zero(),
                        noVelocityNoise));
    }

    void imuPreintegration::reset_graph() {

        // get updated noise before reset
        gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise, updatedVelNoise, updatedBiasNoise;

        try
        {
            updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
            updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
            updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
        }
        catch (...)
        {
            printf("[arise_slam] WARN: Failed to reset graph. No marginal covariance key\n");
            updatedPoseNoise = priorPoseNoise;
            updatedVelNoise = priorVelNoise;
            updatedBiasNoise = priorBiasNoise;
        }

        // reset graph
        resetOptimization();
        // add pose
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                   updatedPoseNoise);
        graphFactors.add(priorPose);
        // add velocity
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                    updatedVelNoise);
        graphFactors.add(priorVel);
        // add bias
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                B(0), prevBias_, updatedBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        key = 1;
    }

    void imuPreintegration::initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose) {

        // 0. initialize system
        resetOptimization();

        while (!imuQueOpt.empty())
        {
            if (imuQueOpt.front().timestamp < currentCorrectionTime - delta_t)
            {
                lastImuT_opt = imuQueOpt.front().timestamp;
                imuQueOpt.pop_front();
            }
            else
                break;
        }


        // initial pose
        prevPose_ = lidarPose.compose(lidar2Imu);

        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                   priorPoseNoise);
        graphFactors.add(priorPose);
        // initial velocity
        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                    priorVelNoise);
        graphFactors.add(priorVel);
        // initial bias
        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        key = 1;
        systemInitialized = true;

    }

    void imuPreintegration::integrate_imumeasurement(double currentCorrectionTime) {
        // 1. integrate imu data and optimize

        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            ImuMsg *thisImu = &imuQueOpt.front();
            double imuTime = thisImu->timestamp;
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 200.0) : (imuTime - lastImuT_opt);
                lastImuT_opt = imuTime;

                if(dt < 0.001 || dt > 0.5)
                    dt = 0.005;

                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x(), thisImu->linear_acceleration.y(), thisImu->linear_acceleration.z()),
                        gtsam::Vector3(thisImu->angular_velocity.x(),    thisImu->angular_velocity.y(),    thisImu->angular_velocity.z()), dt);

                imuQueOpt.pop_front();
            }
            else
                break;
        }

    }


    bool imuPreintegration::build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp) {

        // add laser pose prior factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);

        // insert predicted values
        gtsam::NavState propState_ =
                imuIntegratorOpt_->predict(prevState_, prevBias_);
        auto diff  = curPose.translation() - propState_.pose().translation();

        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose,
                                                     correctionNoise);
        graphFactors.add(pose_factor);
        // add imu factor to graph

        const gtsam::PreintegratedImuMeasurements &preint_imu =
                dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(
                        *imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key),
                                    B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(
                        sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);


        // optimize
        bool systemSolvedSuccessfully = false;
        try {
            optimizer.update(graphFactors, graphValues);
            optimizer.update();
            systemSolvedSuccessfully = true;
        }
        catch (const gtsam::IndeterminantLinearSystemException &) {
            systemSolvedSuccessfully = false;
            printf("[arise_slam] WARN: Update failed due to underconstrained call to isam2 in imuPreintegration\n");
        }

        graphFactors.resize(0);
        graphValues.clear();

        if (systemSolvedSuccessfully) {

            gtsam::Values result = optimizer.calculateEstimate();
            prevPose_ = result.at<gtsam::Pose3>(X(key));
            prevVel_ = result.at<gtsam::Vector3>(V(key));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
            // Reset the optimization preintegration object.
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        }
        return systemSolvedSuccessfully;
    }

    void imuPreintegration::repropagate_imuodometry(double currentCorrectionTime) {
        // 2. after optimization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;

        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && imuQueImu.front().timestamp < currentCorrectionTime - delta_t)
        {
            lastImuQT = imuQueImu.front().timestamp;
            imuQueImu.pop_front();
        }
        // repropagate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                ImuMsg *thisImu = &imuQueImu[i];
                double imuTime = thisImu->timestamp;
                double dt = (lastImuQT < 0) ? (1.0 / 200.0) :(imuTime - lastImuQT);
                lastImuQT = imuTime;

                if(dt < 0.001 || dt > 0.5)
                    dt = 0.005;

                imuIntegratorImu_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x(), thisImu->linear_acceleration.y(), thisImu->linear_acceleration.z()),
                        gtsam::Vector3(thisImu->angular_velocity.x(),    thisImu->angular_velocity.y(),    thisImu->angular_velocity.z()), dt);
                lastImuQT = imuTime;
            }
        }

    }

    void imuPreintegration::process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 relativePose) {

        // reset graph for speed
        if (key > 100) {
            reset_graph();
        }

        // 1. integrate_imumeasurement
        integrate_imumeasurement(currentCorrectionTime);

        lidarodom_w_cur = relativePose;

        // 2. build_graph
        bool successOptimization = build_graph(lidarodom_w_cur, currentCorrectionTime);

        // 3. check optimization
        if (failureDetection(prevVel_, prevBias_) || !successOptimization) {
            printf("[arise_slam] WARN: failureDetected\n");
            resetParams();
            return;
        }

        // 4. reprogate_imuodometry
        repropagate_imuodometry(currentCorrectionTime);
        ++key;

        doneFirstOpt = true;
    }

    bool imuPreintegration::failureDetection(const gtsam::Vector3 &velCur,
                                             const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            printf("[arise_slam] WARN: Large velocity, reset IMU-preintegration!\n");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(),
                           biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(),
                           biasCur.gyroscope().z());

        if (ba.norm() > 2.0 || bg.norm() > 1.0) {
            printf("[arise_slam] WARN: Large bias, reset IMU-preintegration!\n");
            return true;
        }

        return false;
    }

    // Data input: add laser odometry (replaces laserodometryHandler)
    void imuPreintegration::addLaserOdometry(double timestamp, const Eigen::Vector3d& position,
                                              const Eigen::Quaterniond& orientation, double degenerate_flag) {
        std::lock_guard<std::mutex> lock(mBuf);

        auto odom = std::make_shared<ImuOdomMsg>();
        odom->timestamp = timestamp;
        odom->position = position;
        odom->orientation = orientation;
        odom->covariance[0] = degenerate_flag;
        cur_frame = odom;

        double lidarOdomTime = timestamp;

        if (imuQueOpt.empty())
            return;

        float p_x = position.x();
        float p_y = position.y();
        float p_z = position.z();
        float r_x = orientation.x();
        float r_y = orientation.y();
        float r_z = orientation.z();
        float r_w = orientation.w();
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z),
                                              gtsam::Point3(p_x, p_y, p_z));

        // 0. initialize system
        if (systemInitialized == false) {
            initial_system(lidarOdomTime, lidarPose);
            return;
        }

        TicToc Optimization_time;
        //1. process imu odometry
        process_imu_odometry(lidarOdomTime, lidarPose);

#if 1
        // 2. safe landing process
        double latest_imu_time;
        latest_imu_time = imuQueImu.back().timestamp;

        if (lidarOdomTime - latest_imu_time < imu_laser_timedelay) {
            RESULT = IMU_STATE::SUCCESS;
            health_status = true;

          if((int)degenerate_flag == 1)
           {
             RESULT = IMU_STATE::FAIL;
           }

        } else {
            health_status = false;
            if (cur_frame != nullptr && last_frame != nullptr) {
                Eigen::Vector3d velocity_curr;
                double dt_frames = cur_frame->timestamp - last_frame->timestamp;
                if (dt_frames > 1e-6) {
                    velocity_curr.x() = (cur_frame->position.x() - last_frame->position.x()) / dt_frames;
                    velocity_curr.y() = (cur_frame->position.y() - last_frame->position.y()) / dt_frames;
                    velocity_curr.z() = (cur_frame->position.z() - last_frame->position.z()) / dt_frames;
                }

                printf("[arise_slam] LOOSE CONNECTION WITH IMU DRIVER, PLEASE CHECK HARDWARE!!\n");
                RESULT = IMU_STATE::FAIL;
                health_status = false;
            }
        }

        last_frame = cur_frame;
#endif
        last_processed_lidar_time = lidarOdomTime;
    }

    // Data input: add visual odometry (replaces visualodometryHandler)
    void imuPreintegration::addVisualOdometry(double timestamp, const Eigen::Vector3d& position,
                                               const Eigen::Quaterniond& orientation) {
        printf("[arise_slam] visualodometryHandler received\n");
        std::lock_guard<std::mutex> lock(mBuf);
        if (imuQueOpt.empty())
            return;

        auto odom = std::make_shared<ImuOdomMsg>();
        odom->timestamp = timestamp;
        odom->position = position;
        odom->orientation = orientation;
        visualOdomBuf.addMeas(odom, timestamp);
    }


    // IMU data conversion (replaces imuConverter using plain C++ types)
    ImuMsg imuPreintegration::imuConverter(const ImuMsg &imu_in) {
        ImuMsg imu_out = imu_in;

        Eigen::Matrix3d imu_laser_R_Gravity;
        imu_laser_R_Gravity=imu_Init->imu_laser_R_Gravity;

        Eigen::Vector3d rpy;
        rpy=imu_Init->rotationMatrixToRPY(imu_laser_R_Gravity);

        // rotate gyroscope
        Eigen::Vector3d gyr = imu_in.angular_velocity;
        gyr=imu_laser_R_Gravity*gyr;
        imu_out.angular_velocity = gyr;

        // rotate acceleration
        Eigen::Vector3d acc = imu_in.linear_acceleration;
        acc=imu_laser_R_Gravity*acc;
        acc = acc + ((gyr - gyr_pre) * 200).cross(- imu_laser_T) + gyr.cross(gyr.cross(-imu_laser_T));
        imu_out.linear_acceleration = acc;

        // rotate roll pitch yaw
        Eigen::Quaterniond q = imu_in.orientation;
        q.normalize();

        Eigen::Quaterniond q_extrinsic;
        q_extrinsic=Eigen::Quaterniond(imu_laser_R_Gravity);

        Eigen::Quaterniond q_new;
        q_new=q*q_extrinsic;
        q_new.normalize();

        imu_out.orientation = q_new;

        gyr_pre = gyr;

        if (imu_init_success == true) // limit the accelerations if successfully init
        {
            Eigen::Vector3d acc_mean = imu_Init->acc_mean;
            Eigen::Vector3d gyr_mean = imu_Init->gyr_mean;
            acc_mean = imu_Init->imu_laser_R_Gravity * acc_mean;
            gyr_mean = imu_Init->imu_laser_R_Gravity * gyr_mean;
            Eigen::Vector3d acc_diff = acc - acc_mean;
            Eigen::Vector3d gyr_diff = gyr - gyr_mean;

            if (abs(acc_diff(0)) > config_.imu_acc_x_limit){
                acc_diff(0) = acc_diff(0) > 0 ? config_.imu_acc_x_limit : -config_.imu_acc_x_limit;
            }
            if (abs(acc_diff(1)) > config_.imu_acc_y_limit){
                acc_diff(1) = acc_diff(1) > 0 ? config_.imu_acc_y_limit : -config_.imu_acc_y_limit;
            }
            if (abs(acc_diff(2)) > config_.imu_acc_z_limit){
                acc_diff(2) = acc_diff(2) > 0 ? config_.imu_acc_z_limit : -config_.imu_acc_z_limit;
            }

            imu_out.linear_acceleration = acc_mean + acc_diff;
        }

        return imu_out;
    }

    // Data input: add IMU measurement (replaces imuHandler)
    void imuPreintegration::addImuMeasurement(double timestamp, const Eigen::Vector3d& acc_in,
                                               const Eigen::Vector3d& gyro_in, const Eigen::Quaterniond& orientation_in)
    {
        Eigen::Vector3d acc = acc_in;
        Eigen::Vector3d gyro = gyro_in;

        if (config_.lidar_flip)
        {
          acc.y() *= -1.0;
          acc.z() *= -1.0;
          gyro.y() *= -1.0;
          gyro.z() *= -1.0;
        }

        acc.x() += config_.imu_acc_x_offset;
        acc.y() += config_.imu_acc_y_offset;
        acc.z() += config_.imu_acc_z_offset;

        std::lock_guard<std::mutex> lock(mBuf);

        ImuMsg rawImu;
        rawImu.timestamp = timestamp;
        rawImu.linear_acceleration = acc;
        rawImu.angular_velocity = gyro;
        rawImu.orientation = orientation_in;

        ImuMsg thisImu = imuConverter(rawImu);

        // assert(rawImu.linear_acceleration.x() != thisImu.linear_acceleration.x());


        if(config_.sensor==SensorType::LIVOX && imu_init_success == false)
        {
          Imu::Ptr imudata = std::make_shared<Imu>();
          imudata->time = timestamp;
          imudata->acc = acc;
          imudata->gyr = gyro;
          imudata->q_w_i = orientation_in;

          imuBuf.addMeas(imudata, imudata->time);

          double first_imu_time = 0.0;
          imuBuf.getFirstTime(first_imu_time);

          if (imudata->time - first_imu_time > 1.0 && imu_init_success == false)
            {
                imu_Init->imuInit(imuBuf);
                imu_init_success=true;
                imuBuf.clean(imudata->time);
                std::cout<<"IMU Initialization Process Finish! "<<std::endl;
            }
        }

        if(config_.sensor==SensorType::LIVOX)
        {
           double gravity = 9.8105;
           Eigen::Vector3d acc_scaled = thisImu.linear_acceleration;
           acc_scaled = acc_scaled*gravity/imu_Init->acc_mean.norm();
           thisImu.linear_acceleration = acc_scaled;
        }

        if(imu_init_success==false)
        {
            return;
        }

        double imuTime = thisImu.timestamp;
        double dt = (lastImuT_imu < 0) ? (1.0 / 200.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        if(dt < 0.001 || dt > 0.5)
            dt = 0.005;

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        // Extract RPY from orientation using Eigen
        Eigen::Vector3d euler = thisImu.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        double roll = euler[0], pitch = euler[1], yaw = euler[2];

        // Create yaw-zeroed orientation for roll/pitch usage
        Eigen::Quaterniond yaw_quat(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond orientation_zeroed_yaw = yaw_quat * thisImu.orientation;

        // Re-extract after zeroing yaw
        euler = orientation_zeroed_yaw.toRotationMatrix().eulerAngles(0, 1, 2);
        roll = euler[0]; pitch = euler[1]; yaw = euler[2];

        if (doneFirstOpt == false)
            return;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(
                gtsam::Vector3(thisImu.linear_acceleration.x(),
                               thisImu.linear_acceleration.y(),
                               thisImu.linear_acceleration.z()),
                gtsam::Vector3(thisImu.angular_velocity.x(), thisImu.angular_velocity.y(),
                               thisImu.angular_velocity.z()),
                dt);

        // predict odometry
        gtsam::NavState currentState =
                imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        Eigen::Quaterniond state_q(currentState.quaternion().w(), currentState.quaternion().x(),
                                    currentState.quaternion().y(), currentState.quaternion().z());
        Eigen::Vector3d state_euler = state_q.toRotationMatrix().eulerAngles(0, 1, 2);
        roll = state_euler[0]; pitch = state_euler[1]; yaw = state_euler[2];

        Eigen::Quaterniond yaw_quat2(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond final_orientation = yaw_quat2 * orientation_zeroed_yaw;

        // Compute lidar pose in world frame
        Eigen::Quaterniond q_w_curr;
        if (config_.use_imu_roll_pitch)
        {
            q_w_curr = final_orientation;
        }
        else
        {
            q_w_curr = Eigen::Quaterniond(currentState.quaternion().w(), currentState.quaternion().x(),
                                          currentState.quaternion().y(), currentState.quaternion().z());
        }

        gtsam::Rot3 imuRot(q_w_curr);
        gtsam::Pose3 imuPose =
                gtsam::Pose3(imuRot, currentState.position());
        gtsam::Pose3 lidarPoseOpt = imuPose.compose(imu2Lidar);

        Eigen::Vector3d velocity_w_curr(currentState.velocity().x(), currentState.velocity().y(),
                                        currentState.velocity().z());
        Eigen::Vector3d velocity_curr = currentState.quaternion().inverse() * velocity_w_curr;

        Eigen::Quaterniond q_w_lidar(lidarPoseOpt.rotation().toQuaternion().w(), lidarPoseOpt.rotation().toQuaternion().x(),
                                     lidarPoseOpt.rotation().toQuaternion().y(), lidarPoseOpt.rotation().toQuaternion().z());
        q_w_lidar.normalize();

        // Build output state
        latest_state_.timestamp = imuTime;
        latest_state_.position = Eigen::Vector3d(lidarPoseOpt.translation().x(),
                                                  lidarPoseOpt.translation().y(),
                                                  lidarPoseOpt.translation().z());
        latest_state_.orientation = q_w_lidar;
        latest_state_.velocity = velocity_curr;
        latest_state_.angular_velocity = Eigen::Vector3d(
                thisImu.angular_velocity.x() + prevBiasOdom.gyroscope().x(),
                thisImu.angular_velocity.y() + prevBiasOdom.gyroscope().y(),
                thisImu.angular_velocity.z() + prevBiasOdom.gyroscope().z());
        latest_state_.bias = prevBiasOdom;
        latest_state_.gravity = config_.imuGravity;
        latest_state_.health_status = health_status;
        latest_state_.result = RESULT;

        frame_count++;
        if(frame_count%4==0 && output_callback_) {
            output_callback_(latest_state_);
        }

        use_laserodom = false;
        use_visualodom = false;
    }

    ImuState imuPreintegration::getLatestState() const {
        return latest_state_;
    }

} // end namespace arise_slam
