//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>

#include "tictoc.h"
#include "function.h"

class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }

    // Compute the measurement
    void oplusImpl(const double *update) override {
        _estimate += Eigen::Vector3d(update);
    }

    bool read(std::istream &in) override {
        return false;
    }

    bool write(std::ostream &out) const override {
        return false;
    }
};

// The measurement is the difference between the observed and the expected value
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    // Compute the measurement error
    void computeError() override {
        const auto *v = dynamic_cast<const CurveFittingVertex *> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }

    // Compute the Jacobian
    void linearizeOplus() override {
        const auto *v = dynamic_cast<const CurveFittingVertex *> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
    }

    bool read(std::istream &in) override {
        return false;
    }

    bool write(std::ostream &out) const override {
        return false;
    }

public:
    double _x;  // the input variable
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
    std::vector<double> x_data(N), y_data(N);
    //generate data points
    for (int i = 0; i < N; i++) {
        x_data[i] = double(i) / 10000.0;
        y_data[i] = f(x_data[i]);
    }
    //check size of x, y
    std::cout << "size of x: " << x_data.size() << std::endl;
    std::cout << "size of y: " << y_data.size() << std::endl;

    // define graph
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; //Dense solver

    // create solver
    //using Levenberg solver
    std::cout << "Levenberg solver" << '\n';

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer; //optimizer
    optimizer.setAlgorithm(solver); //set solver
    optimizer.setVerbose(true);     //set verbose

    // add vertex
    auto *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);


    for (int i = 0; i < N; i++) {
        auto *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(y_data[i]);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
        optimizer.addEdge(edge);
    }

    tt.tic();

    optimizer.initializeOptimization();
    optimizer.optimize(100);

    std::cout << "CERES optimization time: " << tt.toc_us() << std::endl;

    // print result
    Eigen::Vector3d abc_estimate = v->estimate();
    std::cout << "estimated model: " << abc_estimate.transpose() << std::endl;

    return 0;
}
