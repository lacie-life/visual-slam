//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <random>
#include "tictoc.h"
#include "function.h"

int main() {

    TicToc tt;

    double ar = 1.0, br = 2.0, cr = 1.0; //ground truth value
    double ae = 2.0, be = -1.0, ce = 5.0; //initial guess
    int N = 100; //number of data points
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
        x[i] = double(i) / 100.0;
        y[i] = f(x[i]);
    }
    //check size of x, y
    std::cout << "size of x: " << x.size() << std::endl;
    std::cout << "size of y: " << y.size() << std::endl;

    //show 10 first value of x,y in one line
    for (int i = 0; i < 10; i++) {
        std::cout << "x: " << x[i] << " y: " << y[i] << std::endl;
    }

    //define function with initial guess
    Function f_gn(ae, be, ce, w_sigma);

    int max_iter = 100;
    double cost;
    double cost_prev = 0.0;
    double eps = 1e-9;
    tt.tic();

    for (auto iter = 0; iter < max_iter; iter++) {
        //define Hessian matrix
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); // Hessian = J^T W^{−1} J in Gauss−Newton
        //define bias
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        //init cost
        cost = 0.0;

        //compute Jacobian matrix with N data points
        for (int i = 0; i < N; i++) {
            //compute Jacobian matrix
            Eigen::Vector3d J = f_gn.getJacobian(x[i]);
            //compute error
            double e = y[i] - f_gn.getValue(x[i]);
            //compute Hessian matrix
            H += inv_sigma * inv_sigma * J * J.transpose(); //3x3 matrix
            //compute bias
            b += -inv_sigma * inv_sigma * e * J; //3x1 vector
            //compute cost error
            cost += e * e;
        }
        //solve Hx = b
        Eigen::Vector3d dx_gn = H.ldlt().solve(b);
        if (std::isnan(dx_gn[0]) || std::isnan(dx_gn[1]) || std::isnan(dx_gn[2])) {
            std::cout << "Error: Gauss-Newton method failed" << std::endl;
            break;
        }

        //if inter > 0 and cost - cost_prev very small
        if (iter > 0 && std::abs(cost - cost_prev) < eps) {
            std::cout << "Converged at iteration " << iter << std::endl;
            break;
        }

        //update parameters
        ae += dx_gn[0];
        be += dx_gn[1];
        ce += dx_gn[2];
        //update cost_prev
        cost_prev = cost;
        //update function with new parameters
        f_gn.setParameters(ae, be, ce);

        //print information
        std::cout << "iter: " << iter << " cost: " << cost << std::endl;
        //dx_gn
        std::cout << "dx_gn: " << dx_gn << std::endl;
    }

    tt.toc_us();
    std::cout << "time: " << tt.toc_us() << " us" << std::endl;
    //print result
    std::cout << "result: " << ae << " " << be << " " << ce << std::endl;
    //print error
    std::cout << "error: a " << ar - ae << " b " << br - be << " c " << cr - ce << std::endl;
    //print cost
    std::cout << "cost: " << cost << std::endl;

    return 0;

}

