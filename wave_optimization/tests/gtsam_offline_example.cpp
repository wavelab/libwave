#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include "wave/wave_test.hpp"
#include "wave/vision/dataset.hpp"

namespace wave {

/** Convert pose to gtsam type */
inline gtsam::Pose3 gtsamPoseFromEigen(const Quaternion &q, const Vec3 &p) {
    return gtsam::Pose3{gtsam::Rot3{q}, gtsam::Point3{p}};
}

class GtsamExample : public ::testing::Test {
 protected:
    VOTestDatasetGenerator generator;
    VOTestDataset dataset;
    gtsam::Cal3_S2 kParams{1, 1, 0, 0, 0};

    GtsamExample() {
        generator.camera.image_width = 640;
        generator.camera.image_height = 480;
        generator.camera.K = kParams.matrix();
        generator.camera.hz = 10.0;
        generator.nb_landmarks = 100;
        generator.landmark_x_bounds << -10, 10;
        generator.landmark_y_bounds << -10, 10;
        generator.landmark_z_bounds << -1, 1;
        dataset = generator.generate();
    }
};

TEST_F(GtsamExample, run) {}

}  // namespace wave
