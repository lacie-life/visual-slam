#include <random>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


// The goal is to fit a line of the form y = m x + c
// We provide noisy (x, y) & estimate (m, c)
using namespace std;
using namespace gtsam;

class UnaryFactor: public NoiseModelFactor1<Vector2> {
    double mx_, my_; ///< X and Y measurements

public:
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
            NoiseModelFactor1<Vector2>(model, j), mx_(x), my_(y) {}

    Vector evaluateError(const Vector2& vec,
                         boost::optional<Matrix&> H = boost::none) const {
        if (H) (*H) = (Matrix(1,2)<< -mx_, -1).finished();
        return Vector1(my_ - vec.x()*mx_ - vec.y()) ;
    }
};

int main(int argc, char** argv) {
    double m_true = 3;
    double c_true = -1;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,1.0);

    NonlinearFactorGraph graph;
    for (int i=-100; i<=100; i++) {
        double x = i + distribution(generator);
        double y =  m_true*x + c_true + distribution(generator);
        auto unaryNoise =
                noiseModel::Diagonal::Sigmas(Vector1(1));
        graph.emplace_shared<UnaryFactor>(1, x, y, unaryNoise);
    }
    Values initialEstimate;
    initialEstimate.insert(1, Vector2(0, 0));
    initialEstimate.print("\nInitial Estimate:\n");  // print
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");
    return 0;
}
