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

}  // end of examples namespace
}  // end of ceres namespace
}  // end of wave namespace
