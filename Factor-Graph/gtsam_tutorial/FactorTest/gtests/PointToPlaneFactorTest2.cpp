
#include "gtest/gtest.h"

#include "PointToPlaneFactor2.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>


static gtsam::SharedNoiseModel noise_model(gtsam::noiseModel::Unit::Create(1));

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

gtsam::Pose3 current_pose_ = gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Vector3(-1, 0.2, 0.9));
gtsam::Vector3 current_velocity = gtsam::Vector3(0.43, -0.91, 0.22);

TEST(PointToPlaneFactor, Jacobian) {
    // Create a factor
    lin_estimator::PointToPlaneFactor2 Factor(C(0), preintimumeasurements, plane_params, lidar_point, current_pose_, current_velocity, noise_model);

    gtsam::Pose3 calib = gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.2, 0.1, 0.7), gtsam::Vector3(0.4, -0.3, 0.2));

    // Use the factor to calculate the Jacobians
    gtsam::Matrix H1Actual;
    Factor.computeErrorAndJacobians(calib, H1Actual);

    boost::function<gtsam::Vector(const gtsam::Pose3&)> f
            = boost::bind(&lin_estimator::PointToPlaneFactor2::evaluateError, Factor, _1, boost::none);
    // Use numerical derivatives to calculate the Jacobians
    gtsam::Matrix H1Expected;
    H1Expected = gtsam::numericalDerivative11(f, calib);

    std::cout << "H1Expected" << std::endl;
    std::cout << H1Expected << std::endl;
    std::cout << "H1Actual" << std::endl;
    std::cout << H1Actual << std::endl << std::endl;

    EXPECT_TRUE(gtsam::assert_equal(H1Expected, H1Actual, 1e-9));
}



