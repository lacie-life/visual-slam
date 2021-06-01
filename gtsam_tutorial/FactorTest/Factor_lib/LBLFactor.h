

#ifndef CPP_LBLFACTOR_H_
#define CPP_LBLFACTOR_H_

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>


using namespace gtsam;

namespace gtsamexamples {

/**
 * Prior on velocity in a Body frame or Geographic frame .
 * Possibilities include:
 *   ENU: East-North-Up navigation frame at some local origin
 *   NED: North-East-Down navigation frame at some local origin
 *   Body: body  frame, origin at vechile's center
 * See Farrell08book or e.g. http://www.dirsig.org/docs/new/coordinates.html
 * @addtogroup Navigation
 */
class LBLFactor: public NoiseModelFactor2<Pose3,Vector1> {

private:

	typedef NoiseModelFactor2<Pose3,Vector1> Base;
	double nT_;
	Point3 pos_;


public:

	/// shorthand for a smart pointer to a factor
	typedef boost::shared_ptr<LBLFactor> shared_ptr;

	/// Typedef to this class
	typedef LBLFactor This;

	/// default constructor - only use for serialization
//	LBLFactor() :nT_(0, 0, 0) {}

//	virtual ~LBLFactor() {}

	/**
	 * @param posekey of the Pose3 variable that will be constrained
	 * @param LBLIn measurement already in navigation coordinates
	 * @param model Gaussian noise model
	 */
	LBLFactor(Key posekey,Key deltakey, const double& LBLIn, const Point3& LBLPos, SharedNoiseModel& model) :
			Base(model, posekey, deltakey), nT_(LBLIn), pos_(LBLPos) {
	}

	/// vector of errors
	Vector evaluateError(const Pose3& p,const Vector1& delta,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none) const;

}; // LBLFactor

}/// namespace gtsamexamples


#endif /* LBLFACTOR_H_ */
