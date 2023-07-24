//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "function.h"
#include "tictoc.h"

class CurveFittingFactor: public gtsam::NoiseModelFactor1<gtsam::Point3> {
    double mx_, my_; // measured x[i] and y[i]

public:
    typedef boost::shared_ptr<CurveFittingFactor> shared_ptr;

    CurveFittingFactor(gtsam::Key key, double mx, double my, const gtsam::SharedNoiseModel& model) :
    NoiseModelFactor1<gtsam::Point3>(model, key), mx_(mx), my_(my) {}

    ~CurveFittingFactor() override = default;

    gtsam::Vector evaluateError(const gtsam::Point3& p, boost::optional<gtsam::Matrix&> H = boost::none) const override {
        gtsam::Vector error(1); // error vector[1]
        // error = my - exp(mx^2 * x + mx * y + z)
        error(0) = my_ - std::exp(p.x() * mx_ * mx_ + p.y() * mx_ + p.z());
        if (H) {// if H is not empty
            // error is a column vector [1xn] = [e1, e2, ..., en]
            // Jacobian is computed as,
            // J = [de1/da, de1/db, de1/dc
            //      de2/da, de2/db, de2/dc]
            //      de3/da, de3/db, de3/dc]

            *H = gtsam::Matrix::Zero(1, 3); //Jacobian matrix H[1x3] = [de/da, de/db, de/dc]
            double y = exp(p.x()  * mx_ * mx_ + p.y()  * mx_ + p.z() ); // output = exp(mx^2 * x + mx * y + z)
            // e = y- output = y - exp(mx^2 * x + mx * y + z)
            (*H)(0, 0) = -mx_ * mx_ * y; //de/da
            (*H)(0, 1) = -mx_ * y; //de/db
            (*H)(0, 2) = -y; //de/dc
        }
        return error;
    }

    // The second is a 'clone' function that allows the factor to be copied. Under most
    // circumstances, the following code that employs the default copy constructor should
    // work fine.

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new CurveFittingFactor(*this))); }

};

int main(){

    TicToc tt;

    double ar = 1.0, br = 2.0, cr = 1.0; //ground truth value
    double ae = 2.0, be = -1.0, ce = 5.0; //initial guess
    int N = 10000; //number of data points
    double w_sigma = 1.0; //sigma of noise
    double inv_sigma = 1.0 / w_sigma; //inverse of sigma

    //define function ground truth
    Function f(ar, br, cr, w_sigma);
    //get value of function at x = 1, y = 1
    std::cout << f(1.0) << std::endl;

    //define vector of data points x, y
    std::vector<double> x(N), y(N);
    //generate data points
    for (int i = 0; i < N; i++) {
        x[i] = double(i) / 10000.0;
        y[i] = f(x[i]);
    }
    //check size of x, y
    std::cout << "size of x: " << x.size() << std::endl;
    std::cout << "size of y: " << y.size() << std::endl;

    //******************************************************************************************
    std::cout << "****************************************************************" << std::endl;

    // 1. Create a factor graph container and add factors to it
    gtsam::NonlinearFactorGraph graph;

    gtsam::Values initialEstimate;
    initialEstimate.insert(1, gtsam::Point3(ae, be, ce));
    initialEstimate.print();

    auto CurveNoise =
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1));

    for(auto i = 0; i < N; i++){
        graph.emplace_shared<CurveFittingFactor>(1, x[i], y[i], CurveNoise);
    }
    // 2. Create an optimizer and optimize
    //create optional parameters
    gtsam::LevenbergMarquardtParams params;
    //set max iterations
    params.setMaxIterations(100);

    tt.tic();
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
    //set maxIterations = 100

    gtsam::Values result = optimizer.optimize();

    std::cout << "time optimize: " << tt.toc_us() << " us"<< std::endl;

    result.print("Final Result:\n");

    // 5. Calculate and print marginal covariances for all variables
    gtsam::Marginals marginals(graph, result);
    std::cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << '\n';

    return 0;

}

