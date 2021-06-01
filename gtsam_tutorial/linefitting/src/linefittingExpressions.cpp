#include <random>

// The two new headers that allow using our Automatic Differentiation Expression framework
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// The goal is to fit a line of the form y = m x + c
// I provide noisy (x, y) and estimate (m, c)

using namespace std;
using namespace gtsam;

// The following is the implementation of an equivalent of
// residual used in ceres solver.
// For curve fitting I cannot think of a way of separating
// h(x) and z
// So, here the r returned is basically the residual which
// is r = h(x) - z, the goal of optimization is to find
// the state variables which makes r = 0, or more precisely the
// summation of all r = 0.

double line(const Vector2& mc, const Vector2& xy,
            OptionalJacobian<1, 2> Hmc,
            OptionalJacobian<1, 2> Hxy) {
    double r = xy.y() - mc.x() * xy.x() - mc.y();
    if(Hmc)
        *Hmc << -xy.x(), -1;
    if(Hxy)
        *Hxy << -mc.x(), 1;
    return r;
}

int main(int argc, char** argv) {
    double m_true = -2;
    double c_true = 2;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,1.0);

    Expression<Vector2> mc(1);
    ExpressionFactorGraph graph_expression;

    auto model = noiseModel::Diagonal::Sigmas(Vector1(2));
    for (int i=-100; i<=100; i++) {
        double x = distribution(generator);
        double y =  m_true*x + c_true + distribution(generator);
        Expression<Vector2> xy(Vector2(x, y));
        auto h = Expression<double>(& line, mc, xy);
        graph_expression.addExpressionFactor(h, 0.0, model);
        // The problem minimizes Summation(||h(x)-z||^2), for line fitting
        // it's difficult to separate h(x) and z, so here I have set
        // h -> h(x)-z and z->0
    }
    Values initialValue;
    initialValue.insert(1, Vector2(0, 0));
    Values result = GaussNewtonOptimizer(graph_expression, initialValue).optimize();
    result.print("Final Result:\n");
    return 0;
}