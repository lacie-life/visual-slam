

#include "gtest/gtest.h"

#include "LBLFactor.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>


using namespace std;
using namespace gtsam;
using namespace gtsamexamples;
// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(1));
// Keys are deliberately *not* in sorted order to test that case.

Key poseKey(2);
Key deltaKey(1);
Point3 point(-2.0, 11.0, 1.0);
double measurement(10.0);
Vector1 deltaR(1.0);

// *************************************************************************
/* ************************************************************************* */
Vector factorError3D(const Pose3& pose, const Vector1& delta,
    const LBLFactor& factor) {
  return factor.evaluateError(pose, delta);
}


TEST( LBLFactor, Jacobian ) {
  // Create a factor
	LBLFactor factor(poseKey, deltaKey, measurement, point, model);

  // Set the linearization point
  Pose3 pose(Rot3::RzRyRx(0.2, -0.3, 1.75), Point3(1.0, 2.0, -3.0));

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  factor.evaluateError(pose, deltaR, H1Actual, H2Actual);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  boost::function<gtsam::Vector(const Pose3&, const Vector1&)> f
            = boost::bind(&LBLFactor::evaluateError, factor, _1, _2, boost::none, boost::none);
  H1Expected = numericalDerivative21(f, pose, deltaR);
  H2Expected = numericalDerivative22(f, pose, deltaR);

  // Verify the Jacobians are correct
  EXPECT_TRUE(assert_equal(H1Expected, H1Actual, 1e-9));
  EXPECT_TRUE(assert_equal(H2Expected, H2Actual, 1e-9));
}





