#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorGraph.hpp"
#include "wave/optimization/ceres/graph_wrapper.hpp"
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
            const auto angle = M_PI_2 + atan2(y, x);

            // Store ground truth
            this->true_poses.emplace_back(Vec3{x, y, angle});

            // Take noisy measurements of landmarks
            this->takeMeasurements(this->true_poses.back().value);
        }
    }

    // Store measurements of landmarks from the given pose
    void takeMeasurements(const Pose2D &pose) {
        this->measurements.emplace_back();

        for (auto k = 0u; k < this->true_landmarks.size(); ++k) {
            Vec2 result;
            measureRangeBearing(pose,
                                this->true_landmarks[k].value,
                                ResultOut<2>{result.data()},
                                JacobianOut<2, 3>{nullptr},
                                JacobianOut<2, 2>{nullptr});

            // Store a measurement for all landmarks for now
            // Add noise
            Vec2 stddev = Vec2{0.05, 0.01};
            Vec2 noisy = result + stddev.unaryExpr(&gaussianNoise);
            auto meas = FactorMeasurement<RangeBearing>{noisy, stddev};
            this->measurements.back().emplace(k, meas);
        }
    }

    // Make an estimate for a landmark position using the first pose it was
    // measured from
    Vec2 getLandmarkEstimate(const Pose2D &pose, const RangeBearing &meas) {
        auto angle = pose.orientation + meas.bearing;
        return pose.position + Vec2{cos(angle), sin(angle)};
    }

    // Ground truth landmark positions, held in zero-noise "measurements"
    std::vector<FactorMeasurement<Landmark2D, void>> true_landmarks;

    // Ground truth poses at each time step
    std::vector<FactorMeasurement<Pose2D, void>> true_poses;

    // Landmark measurements
    // At each time step, there is a map from landmark index to measurements
    std::vector<std::map<int, FactorMeasurement<RangeBearing>>> measurements;
};

TEST_F(GraphSlam2d, example) {
    auto graph = FactorGraph{};

    // Make variables for each pose and landmark
    auto poses = std::vector<std::shared_ptr<Pose2DVar>>{};
    auto landmarks = std::vector<std::shared_ptr<Landmark2DVar>>{};
    for (auto i = 0u; i < this->true_poses.size(); ++i) {
        // Initialize the variable with an estimated pose
        // We need reasonable estimates for the algorithm to succeed, because
        // the problem is not convex. Typically this might come from a motion
        // model, but for now just use the noisy truth.
        const auto initial_estimate =
          this->true_poses[i].value.asVector() + 5 * Vec3::Random();
        poses.push_back(std::make_shared<Pose2DVar>(initial_estimate));
    }
    for (auto i = 0u; i < this->true_landmarks.size(); ++i) {
        // Initialize the variable with an estimated pose
        // For now, get the estimate from the first measurement and the first
        // pose estimate
        const auto &meas = this->measurements[0].at(i).value;
        auto estimate = this->getLandmarkEstimate(poses[0]->value, meas);
        landmarks.push_back(std::make_shared<Landmark2DVar>(estimate));
    }

    // Add factors for each measurement
    for (auto i = 0u; i < this->true_poses.size(); ++i) {
        for (const auto &meas : this->measurements[i]) {
            graph.addFactor(measureRangeBearing,
                            meas.second,
                            poses[i],
                            landmarks[meas.first]);
        }
    }

    // Add one more factor to constrain the first pose at the origin
    graph.addPerfectPrior(this->true_poses[0].value.asVector(), poses[0]);

    // Finally, evaluate the whole graph
    evaluateGraph(graph);

    // Check the results
    // Note the tolerances are loose and arbitrary. This is not a strict test of
    // the algorithm's accuracy, just a check that it's working at all.
    for (auto i = 0u; i < this->true_poses.size(); ++i) {
        const auto &truth = this->true_poses[i].value;
        const auto &estimate = poses[i]->value;
        EXPECT_PRED3(
          VectorsNearWithPrec, truth.position, estimate.position, 0.1)
          << "pose #" << i;
        EXPECT_NEAR(truth.orientation, estimate.orientation, 0.04) << "pose #"
                                                                   << i;
    }

    for (auto i = 0u; i < this->true_landmarks.size(); ++i) {
        const auto &truth = this->true_landmarks[i].value.position;
        const auto &estimate = landmarks[i]->value.position;
        EXPECT_PRED3(VectorsNearWithPrec, truth, estimate, 0.1) << "landmark #"
                                                                << i;
    }
}

}  // namespace wave
