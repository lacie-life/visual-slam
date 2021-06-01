/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file OdometryExample.cpp
 * @brief Simple robot motion example, with prior and two odometry measurements
 * @author Frank Dellaert
 */

/**
 * Example of a simple 2D localization example
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam;


class UnaryFactor: public NoiseModelFactor1<Pose2> {
    double mx_, my_; ///< X and Y measurements

public:
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
            NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

    Vector evaluateError(const Pose2& q,
                         boost::optional<Matrix&> H = boost::none) const
    {
        if (H) (*H) = (Matrix(2,3)<< 1.0,0.0,0.0, 0.0,1.0,0.0).finished();
        return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
    }
};

int main(int argc, char** argv) {

    // Create an empty nonlinear factor graph
    NonlinearFactorGraph graph;

    // Add a prior on the first pose, setting it to the origin
    // A prior factor consists of a mean and a noise model (covariance matrix)
    Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector3(1, 1, 1)));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

    // Add odometry factors
    Pose2 odometry(4.0, 0.15, 0.0);
    // For simplicity, we will use the same noise model for each odometry factor
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector3(0.3, 0.3, 0.1)));
    // Create odometry (Between) factors between consecutive poses
    graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
    graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));
    graph.print("\nFactor Graph:\n"); // print


    // Add GPS factors
    noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));
    graph.add(boost::make_shared<UnaryFactor>(1, 0, 0, unaryNoise));
    graph.add(boost::make_shared<UnaryFactor>(2, 5, 0, unaryNoise));
    graph.add(boost::make_shared<UnaryFactor>(3, 10, 0, unaryNoise));

    // Create the data structure to hold the initialEstimate estimate to the solution
    // For illustrative purposes, these have been deliberately set to incorrect values
    Values initial;
    initial.insert(1, Pose2(0, 0.0, 0.0));
    initial.insert(2, Pose2(0, 0.0, 0.0));
    initial.insert(3, Pose2(0, 0.0, 0.0));
    initial.print("\nInitial Estimate:\n"); // print

    // optimize using Levenberg-Marquardt optimization
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final Result:\n");

    // Calculate and print marginal covariances for all variables
//    cout.precision(2);
//    Marginals marginals(graph, result);
//    cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
//    cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
//    cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

    return 0;
}
