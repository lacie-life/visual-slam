//
// Created by lacie-life on 2023/24/07.   
//

#include <random>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

//define function class
class Function {

public:
    //constructor
    Function(double a, double b, double c, double sigma = 1.0) : a_(a), b_(b), c_(c), sigma_(sigma) {}

    //destructor
    ~Function() = default;

    //get value of function at x, y
    double getValue(double x) const {
        return exp(a_ * x * x + b_ * x + c_);
    }

    //set new parameters
    void setParameters(double a, double b, double c) {
        a_ = a;
        b_ = b;
        c_ = c;
    }

    //operator overload
    double operator()(double x) const {
        //generate random number
        std::random_device rd;
        std::mt19937 gen(rd());
        //normal distribution with sigma = 0.1
        std::normal_distribution<> d(0.0, this->sigma_);
        //return value of function
        return getValue(x) + d(gen);
    }

    //compute Jacobian matrix
    Eigen::Vector3d getJacobian(double x) const {
        Eigen::Vector3d J;
        J[0] = -x * x * exp(a_ * x * x + b_ * x + c_); //dF/da
        J[1] = -x * exp(a_ * x * x + b_ * x + c_); //dF/db
        J[2] = -exp(a_ * x * x + b_ * x + c_); //dF/dc
        return J;
    }

private:
    double a_, b_, c_;
    double sigma_;

};
