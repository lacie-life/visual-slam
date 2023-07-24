

#include "LBLFactor.h"

using namespace gtsam;

namespace gtsamexamples {

//***************************************************************************
Vector LBLFactor::evaluateError(const Pose3& p,const Vector1& delta,
		boost::optional<Matrix&> H1,boost::optional<Matrix&> H2) const {

	double range = p.range(pos_);
	Vector3 pos_b = p.translation();
    Vector3 pos_a = pos_;
    Vector3 delta_p = pos_b - pos_a;
    delta_p = (p.rotation().inverse()).matrix()*delta_p;
	if (H1) {
		*H1 = (Matrix16() << 0,0,0,
						(double)delta_p(0)/range,(double)delta_p(1)/range,(double)delta_p(2)/range).finished();
	}
	double delta_r = delta(0);
	if (H2) { *H2 = (Matrix11() << -1).finished();}
	return (Vector1() <<  range - nT_ - delta_r).finished();

} // end evaluateError

}/// namespace gtsamexamples

