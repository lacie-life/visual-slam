#include "gtest/gtest.h"

#include "PointToPlaneFactor.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>


static gtsam::SharedNoiseModel noise_model(gtsam::noiseModel::Unit::Create(1));

using gtsam::symbol_shorthand::N; // Navstate
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz) // IMU (+ Gyro) Biases
using gtsam::symbol_shorthand::C; // Extrinsic Calibration // Pose3

///// Measurements
gtsam::Vector4 plane_params(2, 1, 6, 3);
gtsam::Vector3 lidar_point(2, 4, 1);

gtsam::Matrix93 H_bias_omega = gtsam::Matrix93::Ones();
gtsam::Matrix93 H_bias_acc = gtsam::Matrix93::Ones();

lin_estimator::PreIntegratedIMUMeasurements preintimumeasurements =
        {gtsam::Rot3::identity(), gtsam::Vector3(0, 0, 0),
         gtsam::Vector3(0, 0, 0), 0.01, gtsam::Vector3(0, 0, -9.81),
                 H_bias_omega, H_bias_acc};

double weight = 1/100.0f;

TEST(PointToPlaneFactor, Jacobian) {
    // Create a factor
    lin_estimator::PointToPlaneFactor Factor(N(0),N(1), B(1), C(0),
                                              preintimumeasurements, plane_params, lidar_point, weight, noise_model);

    gtsam::Pose3 pose1 = gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.3, 0.1, 0.01), gtsam::Vector3(0.5, -0.2, 0.1));
    gtsam::NavState pv1 = gtsam::NavState(pose1, gtsam::Vector3(-0.01, 0.5, 0.3));
    gtsam::Pose3 poseM = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.2, -0.3, 1.75), gtsam::Vector3(1.0, 2.0, -3.0));
    gtsam::Vector3 velocityM = gtsam::Vector3(0.01, 0.2, -0.1);
    gtsam::NavState pvM = gtsam::NavState(poseM, velocityM);
    gtsam::imuBias::ConstantBias biasM = gtsam::imuBias::ConstantBias(gtsam::Vector3(0.001, 0.002, 0.087),
            gtsam::Vector3(0.004, 0.2, 0.03));
    gtsam::Pose3 calib = gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.2, 0.1, 0.7), gtsam::Vector3(0.4, -0.3, 0.2));

    // Use the factor to calculate the Jacobians
    gtsam::Matrix H1Actual, H2Actual, H3Actual, H4Actual;
    Factor.computeErrorAndJacobians(pv1, pvM, biasM, calib, H1Actual, H2Actual, H3Actual, H4Actual);


    boost::function<gtsam::Vector(const gtsam::NavState&, const gtsam::NavState&, const gtsam::imuBias::ConstantBias&, const gtsam::Pose3&)> f
    = boost::bind(&lin_estimator::PointToPlaneFactor::evaluateError, Factor, _1, _2, _3, _4,
            boost::none, boost::none, boost::none, boost::none);
    // Use numerical derivatives to calculate the Jacobians
    gtsam::Matrix H1Expected, H2Expected, H3Expected, H4Expected;
    H1Expected = gtsam::numericalDerivative41(f, pv1, pvM, biasM, calib);
    H2Expected = gtsam::numericalDerivative42(f, pv1, pvM, biasM, calib);
    H3Expected = gtsam::numericalDerivative43(f, pv1, pvM, biasM, calib);
    H4Expected = gtsam::numericalDerivative44(f, pv1, pvM, biasM, calib);

    std::cout << "H1Expected" << std::endl;
    std::cout << H1Expected << std::endl;
    std::cout << "H1Actual" << std::endl;
    std::cout << H1Actual << std::endl << std::endl;

    std::cout << "H2Expected" << std::endl;
    std::cout << H2Expected << std::endl;
    std::cout << "H2Actual" << std::endl;
    std::cout << H2Actual << std::endl << std::endl;

    std::cout << "H3Expected" << std::endl;
    std::cout << H3Expected << std::endl;
    std::cout << "H3Actual" << std::endl;
    std::cout << H3Actual << std::endl << std::endl;

    std::cout << "H4Expected" << std::endl;
    std::cout << H4Expected << std::endl;
    std::cout << "H4Actual" << std::endl;
    std::cout << H4Actual << std::endl << std::endl;

    EXPECT_TRUE(gtsam::assert_equal(H1Expected, H1Actual, 1e-9));
    EXPECT_TRUE(gtsam::assert_equal(H2Expected, H2Actual, 1e-9));
    EXPECT_TRUE(gtsam::assert_equal(H3Expected, H3Actual, 1e-9));
    EXPECT_TRUE(gtsam::assert_equal(H4Expected, H4Actual, 1e-9));
}