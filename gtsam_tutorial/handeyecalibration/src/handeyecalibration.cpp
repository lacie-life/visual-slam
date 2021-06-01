/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HandEyeCalibration AX = XB
 * @brief Example of application of ISAM2 for HandEyeCalibration AX = XB
 */

#include <cstring>
#include <fstream>
#include <iostream>

#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/navigation/ImuFactor.h>

// CERES related includes
using namespace std;
using namespace gtsam;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>

class rotationError {
private:
    // All the given info
    const Eigen::Vector3d axis_imu_;
    const Eigen::Vector3d axis_lidar_;

public:
    rotationError(const Eigen::Vector3d axis_imu,
                  const Eigen::Vector3d axis_lidar):
                  axis_imu_(axis_imu),
                  axis_lidar_(axis_lidar)
    {}

    template  <typename  T>
    bool operator() (const T* const calib_angle,
                     T* residual) const {
        Eigen::Matrix<T, 3, 1> _axis_imu;
        _axis_imu(0) = T(axis_imu_.x());
        _axis_imu(1) = T(axis_imu_.y());
        _axis_imu(2) = T(axis_imu_.z());

        Eigen::Matrix<T, 3, 1> _axis_lidar;
        _axis_lidar(0) = T(axis_lidar_.x());
        _axis_lidar(1) = T(axis_lidar_.y());
        _axis_lidar(2) = T(axis_lidar_.z());

        Eigen::Matrix<T, 3, 1> _axis_lidar_transformed;
        ceres::AngleAxisRotatePoint(calib_angle, _axis_lidar.data(), _axis_lidar_transformed.data());

        residual[0] = _axis_lidar_transformed(0) - _axis_imu(0);
        residual[1] = _axis_lidar_transformed(1) - _axis_imu(1);
        residual[2] = _axis_lidar_transformed(2) - _axis_imu(2);
        return true;
    }
};

struct IMUIntrinsics {
    double accelerometer_sigma;
    double gyroscope_sigma;
    double integration_sigma;
    double accelerometer_bias_sigma;
    double gyroscope_bias_sigma;
    double average_delta_t;
};

struct ImuMeasurement {
    uint32_t time_sec;
    uint32_t time_nsec;
    double timestamp; // time_sec + time_nsec/1e9
    double dt;
    Vector3 accelerometer;
    Vector3 gyroscope;  // omega
};

struct LOPoseMeasurement {
    uint32_t time_sec;
    uint32_t time_nsec;
    double timestamp; // time_sec + time_nsec/1e9
    Vector3 position;  // x,y,z
    Quaternion quat; // qw, qx, qy, qz
};

const string output_filename = "IMULOPoseResults.csv";

string imu_meta_data_filename;
string imu_data_filename;
string lopose_data_filename;

void loadIMULOPoseData(IMUIntrinsics& imu_calibration,
                       vector<ImuMeasurement>& imu_measurements,
                       vector<LOPoseMeasurement>& lopose_measurements) {
    string line;
    // Read IMU metadata AccelerometerBiasSigma GyroscopeBiasSigma AverageDeltaT
    ifstream imu_metadata(imu_meta_data_filename.c_str());
    std::cout <<"-- Reading sensor metadata" << std::endl;
    getline(imu_metadata, line, '\n');  // ignore the first line
//    // Load IMU calibration
    getline(imu_metadata, line, '\n');
    sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf",
            &imu_calibration.accelerometer_sigma, &imu_calibration.gyroscope_sigma, &imu_calibration.integration_sigma,
            &imu_calibration.accelerometer_bias_sigma, &imu_calibration.gyroscope_bias_sigma,
            &imu_calibration.average_delta_t);
    printf("%lf %lf %lf %lf %lf %lf\n",
           imu_calibration.accelerometer_sigma, imu_calibration.gyroscope_sigma, imu_calibration.integration_sigma,
           imu_calibration.accelerometer_bias_sigma, imu_calibration.gyroscope_bias_sigma,
           imu_calibration.average_delta_t);
    // Read IMU data
    // Time dt accelX accelY accelZ omegaX omegaY omegaZ
    printf("-- Reading IMU measurements from file\n"); {
        ifstream imu_data(imu_data_filename.c_str());
        uint32_t time_sec = 0, time_nsec = 0;
        double acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;
        int count = 0;
        double timestamp_prev = 0;
        while (getline(imu_data, line, '\n')) {
            sscanf(line.c_str(), "%u, %u, %lf, %lf, %lf, %lf, %lf, %lf",
                   &time_sec, &time_nsec, &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
            ImuMeasurement measurement;
            measurement.time_sec = time_sec;
            measurement.time_nsec = time_nsec;
            measurement.timestamp = double(time_sec) + double(time_nsec)/1e9;
            measurement.accelerometer = Vector3(acc_x, acc_y, acc_z);
            measurement.gyroscope = Vector3(gyro_x, gyro_y, gyro_z);
            if(count == 0) {
                measurement.dt = measurement.timestamp;
            }
            else {
                measurement.dt = measurement.timestamp - timestamp_prev;
            }
            imu_measurements.push_back(measurement);
            timestamp_prev = measurement.timestamp;
            count++;
        }
        std::cout << "No of IMU measurements: " << imu_measurements.size() << std::endl;
    }
    // Read Lopose data
    // TimeSec,TimeNSec, X,Y,Z, qx, qy, qz, qw
    printf("-- Reading LOPose measurements from file\n"); {
        ifstream lopose_data(lopose_data_filename.c_str());
        uint32_t time_sec = 0, time_nsec = 0;
        double lopose_x = 0, lopose_y = 0, lopose_z = 0;
        double loquat_x = 0, loquat_y = 0, loquat_z = 0, loquat_w = 0;
        while (getline(lopose_data, line, '\n')) {
            sscanf(line.c_str(), "%u, %u, %lf,%lf,%lf, %lf,%lf,%lf,%lf",
                    &time_sec, &time_nsec, &lopose_x, &lopose_y, &lopose_z, &loquat_x, &loquat_y, &loquat_z, &loquat_w);
            LOPoseMeasurement measurement;
            measurement.time_sec = time_sec;
            measurement.time_nsec = time_nsec;
            measurement.timestamp = double(time_sec) + double(time_nsec)/1e9;
            measurement.position = Vector3(lopose_x, lopose_y, lopose_z);
            measurement.quat = Quaternion(loquat_w, loquat_x, loquat_y, loquat_z);
            lopose_measurements.push_back(measurement);
        }
        std::cout << "No of LO Pose measurements: " << lopose_measurements.size() << std::endl;
    }
}

void rotationalHandEyeCalibration(IMUIntrinsics imu_intrinsics,
                                  vector<ImuMeasurement> imu_measurements,
                                  vector<LOPoseMeasurement> lopose_measurements) {
    double g = 9.8; // [TODO: Check if this should ne -ve]
    auto w_coriolis = Vector3::Zero();  // zero vector

    // Set IMU preintegration parameters
    auto current_bias = imuBias::ConstantBias();  // zero bias
    Matrix33 measured_acc_cov = I_3x3 * pow(imu_intrinsics.accelerometer_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(imu_intrinsics.gyroscope_sigma, 2);
    // error committed in integrating position from velocities
    Matrix33 integration_error_cov = I_3x3 * pow(imu_intrinsics.integration_sigma, 2);

    auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(g);
    imu_params->accelerometerCovariance = measured_acc_cov;     // acc white noise in continuous
    imu_params->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
    imu_params->gyroscopeCovariance = measured_omega_cov;       // gyro white noise in continuous
    imu_params->omegaCoriolis = w_coriolis;

    std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement = nullptr;

    ceres::Problem problem_rot;
    ceres::LossFunction *loss_function_rot = NULL;
    Eigen::Vector3d calib_angle = Eigen::Vector3d::Zero();
    for(size_t i = 1; i < lopose_measurements.size() ; i++) {
        double timestamp_prev = lopose_measurements[i-1].timestamp;
        double timestamp_curr = lopose_measurements[i].timestamp;
        auto lopose_prev = Pose3(lopose_measurements[i-1].quat, lopose_measurements[i-1].position);
        auto lopose_curr = Pose3(lopose_measurements[i].quat, lopose_measurements[i].position);
        auto delta_lo_pose = (lopose_prev.inverse())*lopose_curr;
        Rot3 deltaRij_L = delta_lo_pose.rotation();
        Eigen::Matrix3d deltaRij_L_eig;
        deltaRij_L_eig << deltaRij_L.r1().x(), deltaRij_L.r2().x(), deltaRij_L.r3().x(),
                deltaRij_L.r1().y(), deltaRij_L.r2().y(), deltaRij_L.r3().y(),
                deltaRij_L.r1().z(), deltaRij_L.r2().z(), deltaRij_L.r3().z();

        current_summarized_measurement = std::make_shared<PreintegratedImuMeasurements>(imu_params, current_bias);
        size_t j = 0;
        while(j < imu_measurements.size() && imu_measurements[j].timestamp <= timestamp_curr) {
            if(imu_measurements[j].timestamp >= timestamp_prev) {
                current_summarized_measurement->integrateMeasurement(imu_measurements[j].accelerometer,
                                                                     imu_measurements[j].gyroscope,
                                                                     imu_measurements[j].dt);
            }
            j++;
        }
        std::cout << i << "\t" << current_summarized_measurement->deltaPij().transpose() << std::endl;
        Rot3 deltaRij_I = current_summarized_measurement->deltaRij();
        Eigen::Matrix3d deltaRij_I_eig;
        deltaRij_I_eig << deltaRij_I.r1().x(), deltaRij_I.r2().x(), deltaRij_I.r3().x(),
                deltaRij_I.r1().y(), deltaRij_I.r2().y(), deltaRij_I.r3().y(),
                deltaRij_I.r1().z(), deltaRij_I.r2().z(), deltaRij_I.r3().z();

        Eigen::Vector3d axisangle_imu;
        ceres::RotationMatrixToAngleAxis(deltaRij_I_eig.data(), axisangle_imu.data());
        Eigen::Vector3d axisangle_lidar;
        ceres::RotationMatrixToAngleAxis(deltaRij_L_eig.data(), axisangle_lidar.data());

        ceres::CostFunction *cost_function_rot =
                new ceres::AutoDiffCostFunction<rotationError, 3, 3>
                        (new rotationError(axisangle_imu, axisangle_lidar));
        problem_rot.AddResidualBlock(cost_function_rot, loss_function_rot, calib_angle.data());
    }
    ceres::Solver::Options options_rot;
    options_rot.linear_solver_type = ceres::DENSE_QR;
    options_rot.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options_rot, &problem_rot, &summary);
    Eigen::Matrix3d I_R_L;
    ceres::AngleAxisToRotationMatrix(calib_angle.data(), I_R_L.data());
    Eigen::Vector3d euler_RPY = I_R_L.eulerAngles(0, 1, 2)*180/M_PI;
    std::cout << "----- Non Linear Solution: Rotation only -----" << std::endl;
    std::cout << "Roll: " << euler_RPY.x() << std::endl;
    std::cout << "Pitch: " << euler_RPY.y() << std::endl;
    std::cout << "Yaw: " << euler_RPY.z() << std::endl;
    std::cout << I_R_L << std::endl;
}

int main(int argc, char* argv[]) {
    imu_meta_data_filename = argv[1];
    imu_data_filename = argv[2];
    lopose_data_filename = argv[3];

    std::cout << "imu_meta_data_filename: " << imu_meta_data_filename << std::endl;
    std::cout << "imu_data_filename: " << imu_data_filename << std::endl;
    std::cout << "gps_data_filename: " << lopose_data_filename << std::endl;

    IMUIntrinsics imu_intrinsics;
    vector<ImuMeasurement> imu_measurements;
    vector<LOPoseMeasurement> lopose_measurements;
    loadIMULOPoseData(imu_intrinsics, imu_measurements, lopose_measurements);
    rotationalHandEyeCalibration(imu_intrinsics, imu_measurements, lopose_measurements);

    return 1;
}

