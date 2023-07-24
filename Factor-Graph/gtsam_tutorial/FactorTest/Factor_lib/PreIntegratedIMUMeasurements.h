
#ifndef FACTORTEST_PREINTEGRATEDIMUMEASUREMENTS_H
#define FACTORTEST_PREINTEGRATEDIMUMEASUREMENTS_H
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ImuBias.h>

using namespace gtsam;
namespace lin_estimator {
    struct PreIntegratedIMUMeasurements{
        Rot3 deltaR;
        Vector3 deltaV;
        Vector3 deltaP;
        double deltaT;
        Vector3 gravity;
        Matrix93 H_bias_omega;
        Matrix93 H_bias_accel;
    };
}
#endif //FACTORTEST_PREINTEGRATEDIMUMEASUREMENTS_H
