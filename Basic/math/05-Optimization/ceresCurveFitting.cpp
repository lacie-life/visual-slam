//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <random>
#include "tictoc.h"
#include "function.h"

// Define a functor for the cost function.
struct CURVE_FITTING_COST {
    CURVE_FITTING_COST(double x, double y)
            : x_(x), y_(y) {}

    template<typename T>
    bool operator()(const T *const abc, T *residuals) const {
        residuals[0] = T(y_) - ceres::exp(abc[0] * T(x_) * T(x_) + abc[1] * T(x_) + abc[2]);
        return true;
    }

    double x_;
    double y_;
};

int main() {
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

    double abc[3] = {ae, be, ce};

    //ceres problem
    ceres::Problem problem;
    for (int i = 0; i < N; i++) {
        //add cost function
        problem.AddResidualBlock( //add residual block
                new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(//cost function, vars 3, out 1
                        new CURVE_FITTING_COST(x[i], y[i])),//cost function as functor
                nullptr, //no loss function
                abc); //variable to be optimized
    }

    //solving options
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //solver type CHOLESKY
    options.minimizer_progress_to_stdout = true; //print progress to stdout
    options.max_num_iterations = 100; //max number of iterations

    //solve summary
    ceres::Solver::Summary summary;
    //titic
    tt.tic();
    //solve
    ceres::Solve(options, &problem, &summary); //solve problem
    tt.toc_us();
    //time
    std::cout << "time: " << tt.toc_us() << " us" << std::endl;
    std::cout << summary.BriefReport() << std::endl;
    //print result
    std::cout << "estimated abc: " << abc[0] << ", " << abc[1] << ", " << abc[2] << '\n';

    return 0;
}