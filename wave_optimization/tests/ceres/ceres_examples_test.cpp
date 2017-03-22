#include <functional>

#include <gtest/gtest.h>

#include "wave/utils/utils.hpp"
#include "wave/optimization/ceres/ceres_examples.hpp"


namespace wave {
namespace ceres {
namespace examples {

TEST(CeresExamples, AutoDiffExample) {
    runAutoDiffExample();
}

TEST(CeresExamples, NumericalDiffExample) {
    runNumericalDiffExample();
}

TEST(CeresExamples, AnalyticalDiffExample) {
    runAnalyticalDiffExample();
}

}  // end of examples namespace
}  // end of ceres namespace
}  // end of wave namespace
