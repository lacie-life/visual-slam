#include "gtsam_backend/graph.h"
#include "estimator/parameters.h"

bool graph_solver::set_imu_preintegration(const gtsam::State& init_state) {

    //Create GTSAM preintegration parameters for use with Foster's version
    auto params = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
                gtsam::Vector3(0.0, 9.8, 0.0));
    /*boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params;
    params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.8); */ // Z-up navigation frame: gravity points along negative Z-axis !!!

    params->setAccelerometerCovariance(gtsam::I_3x3 * pow(0.0565, 2.0));  // acc white noise in continuous
    params->setGyroscopeCovariance(gtsam::I_3x3 * pow(4.0e-5, 2.0));  // gyro white noise in continuous
    params->biasAccCovariance = pow(0.00002, 2.0) * gtsam::Matrix33::Identity(3,3);  // acc bias in continuous
    params->biasOmegaCovariance = pow(0.001, 2.0) * gtsam::Matrix33::Identity(3,3);  // gyro bias in continuous
    params->setIntegrationCovariance(gtsam::I_3x3 * 0.1);  // error committed in integrating position from velocities
    params->biasAccOmegaInt = 1e-5*gtsam::Matrix66::Identity(6,6); // error in the bias used for preintegration

    //    gtsam::Rot3 iRb(1, 0, 0,
    //                    0, 1, 0,
    //                       0, 0, 1);
    // body to IMU translation (meters)
    //gtsam::Point3 iTb(-0.006, 0.005, 0.012);

    //params->body_P_sensor = gtsam::Pose3(iRb, iTb);

    gtsam::Vector3 acc_bias(0.0, -0.0942015, 0.0);  // in camera frame
    gtsam::Vector3 gyro_bias(-0.00527483, -0.00757152, -0.00469968);
    //gtsam::imuBias::ConstantBias imu_bias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);

    // Actually create the GTSAM preintegration
    preint_gtsam = new gtsam::PreintegratedCombinedMeasurements(params, init_state.b());
    return true;
}

gtsam::CombinedImuFactor graph_solver::createIMUFactor(double update_time)
{
    imu_lock_.lock();
    while(acc_vec_.size() > 1 && acc_vec_.front().first <= update_time)
    {
        double dt = acc_vec_.at(1).first - acc_vec_.at(0).first;

        //std::cout << "dt: " << dt << std::endl;
        if(dt > 0)
        {
            Eigen::Vector3d meas_acc;
            Eigen::Vector3d meas_ang_vel;
            meas_acc        = acc_vec_.at(0).second;
            meas_ang_vel    = ang_vel_vec_.at(0).second;
            preint_gtsam->integrateMeasurement(meas_acc, meas_ang_vel, dt);
            //            std::cout << "meas_acc: " << meas_acc << std::endl;
            //            std::cout << "meas_ang_vel: " << meas_ang_vel << std::endl;
        }

        acc_vec_.erase(acc_vec_.begin());
        ang_vel_vec_.erase(ang_vel_vec_.begin());
    }
    double dt_f = update_time - acc_vec_.at(0).first;

    if(dt_f > 0)
    {
        Eigen::Vector3d meas_acc;
        Eigen::Vector3d meas_ang_vel;
        meas_acc        = acc_vec_.at(0).second;
        meas_ang_vel    = ang_vel_vec_.at(0).second;
        preint_gtsam->integrateMeasurement(meas_acc, meas_ang_vel, dt_f);
        acc_vec_.at(0).first = update_time;
        ang_vel_vec_.at(0).first = update_time;
    }
    imu_lock_.unlock();

    return gtsam::CombinedImuFactor(X(cur_sc_),  V(cur_sc_),
                                    X(cur_sc_+1), V(cur_sc_+1),
                                    B(cur_sc_), B(cur_sc_ +1), *preint_gtsam);
}

gtsam::State graph_solver::getPredictedState(gtsam::Values prev_values)
{
    // Get the current state (t=k)
    gtsam::State stateK = gtsam::State(prev_values.at<gtsam::Pose3>(X(cur_sc_)),
                                       prev_values.at<gtsam::Vector3>(V(cur_sc_)),
                                       prev_values.at<gtsam::Bias>(B(cur_sc_)));

    // From this we should predict where we will be at the next time (t=K+1)
    gtsam::NavState stateK1 = preint_gtsam->predict(gtsam::NavState(stateK.pose(), stateK.v()), stateK.b());
    return gtsam::State(stateK1.pose(), stateK1.v(), stateK.b());
}

void graph_solver::resetIMUIntegration()
{
    if(values_prev_.exists(V(cur_sc_)))
    {
        preint_gtsam->resetIntegrationAndSetBias(values_prev_.at<gtsam::Bias>(B(cur_sc_)));
    }

    return;
}
