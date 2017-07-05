#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorGraph.hpp"
#include "example_instances.hpp"

namespace wave {

/** Draw a single number from zero-mean Gaussian distribution */
inline double gaussianNoise(double stddev) {
    static std::default_random_engine generator{std::random_device{}()};
    static std::normal_distribution<> normal{0, 1};

    return stddev * normal(generator);
}

class GraphSlam2d : public ::testing::Test {
    const int num_landmarks = 10;
    const double circle_radius = 5;
    const int num_steps = 5;
    const double time_per_step = 0.05;

 protected:
    GraphSlam2d() {
        // Generate landmark positions
        for (auto i = 0; i < this->num_landmarks; ++i) {
            auto pos = Vec2{7.5, i};
            this->true_landmarks.emplace_back(pos);
        }

        // Simulate robot motion
        for (auto i = 0; i < this->num_steps; ++i) {
            const auto t = this->time_per_step * i;

            // Generate ground truth circular path
            const auto x = this->circle_radius * cos(t);
            const auto y = this->circle_radius * sin(t);
            //            const auto angle = M_PI_2 + atan2(y, x);
            const auto angle = 0;

            // Store ground truth
            this->true_poses.emplace_back(Pose2D<double>{{x, y}, angle});

            // Take noisy measurements of landmarks
            this->takeMeasurements(this->true_poses.back().value);
        }
    }

    // Store measurements of landmarks from the given pose
    void takeMeasurements(const Pose2D<double> &pose) {
        this->measurements.emplace_back();

        for (auto k = 0u; k < this->true_landmarks.size(); ++k) {
            RangeBearing<double> result;
            RangeBearingMeasurementFunctor::evaluate(
              pose, this->true_landmarks[k].value, result);

            // Store a measurement for all landmarks for now
            // Add noise
            RangeBearing<double> stddev{0.05, 0.01};
            RangeBearing<double> noisy =
              result.matrix() + stddev.matrix().unaryExpr(&gaussianNoise);
            auto meas = FactorMeasurement<RangeBearing>{noisy, stddev};
            this->measurements.back().emplace(k, meas);
        }
    }

    // Ground truth landmark positions, held in zero-noise "measurements"
    std::vector<FactorMeasurement<Position2D, void>> true_landmarks;

    // Ground truth poses at each time step
    std::vector<FactorMeasurement<Pose2D, void>> true_poses;

    // Landmark measurements
    // At each time step, there is a map from landmark index to measurements
    std::vector<std::map<int, FactorMeasurement<RangeBearing>>> measurements;
};

TEST_F(GraphSlam2d, example) {
    FactorGraph graph;

    // Make variables for each pose and landmark
    auto poses = std::vector<std::shared_ptr<Pose2DVar>>{};
    auto landmarks = std::vector<std::shared_ptr<Landmark2DVar>>{};
    for (auto i = 0u; i < this->true_poses.size(); ++i) {
        poses.push_back(std::make_shared<Pose2DVar>());
    }
    for (auto i = 0u; i < this->true_landmarks.size(); ++i) {
        // @todo: the variables are still initialized to zero, and the distance
        // measurement has undefined jacobian singular at 0... manually
        // initialize them non-zero for now.
        landmarks.push_back(std::make_shared<Landmark2DVar>(Vec2{1., 1.}));
    }

    // Add factors for each measurement
    for (auto i = 0u; i < this->true_poses.size(); ++i) {
        for (const auto &meas : this->measurements[i]) {
            graph.addFactor<RangeBearingMeasurementFunctor>(
              meas.second, poses[i], landmarks[meas.first]);
        }
    }

    // Add one more factor to constrain the first pose at the origin
    const auto prior = Pose2D<double>{{5, 0}, 0};
    graph.addPerfectPrior(prior, poses[0]);

    // Finally, evaluate the whole graph
    graph.evaluate();

    // Check the results
    for (auto i = 0u; i < this->true_poses.size(); ++i) {
        const auto &truth = this->true_poses[i].value.position;
        const auto &estimate = poses[i]->value.position;
        EXPECT_PRED3(VectorsNearWithPrec, truth, estimate, 0.1) << "pose #"
                                                                << i;
    }

    for (auto i = 0u; i < this->true_landmarks.size(); ++i) {
        const auto &truth = this->true_landmarks[i].value;
        const auto &estimate = landmarks[i]->value;
        EXPECT_PRED3(VectorsNearWithPrec, truth, estimate, 0.1) << "landmark #"
                                                                << i;
    }
}

}  // namespace wave
