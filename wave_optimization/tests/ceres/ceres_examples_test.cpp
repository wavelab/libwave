#include <functional>

#include <gtest/gtest.h>

#include "wave/utils/utils.hpp"
#include "wave/optimization/ceres/ceres_examples.hpp"


namespace wave {
namespace ceres {
namespace examples {

TEST(CeresExamples, runAutoDiffExample) {
    runAutoDiffExample();
}

TEST(CeresExamples, runNumericalDiffExample) {
    runNumericalDiffExample();
}

TEST(CeresExamples, runAnalyticalDiffExample) {
    runAnalyticalDiffExample();
}

TEST(CeresExamples, runCurveFittingExample) {
    runCurveFittingExample();
}

TEST(CeresExamples, eigenMaps) {
    double vec3[3] = {0};

    Eigen::Map<Eigen::Vector3d> vec_map(vec3);

    vec_map(0) = 2;
    vec_map(2) = -2;
}

}  // namespace examples
}  // namespace ceres
}  // namespace wave
