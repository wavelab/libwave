#include <aslam/cameras/RollingShutter.hpp>
#include "sm/assert_macros.hpp"

namespace aslam {
namespace cameras {
RollingShutter::RollingShutter()
    : _lineDelay(0.0) {

}

RollingShutter::RollingShutter(double lineDelay)
    : _lineDelay(lineDelay) {

}

RollingShutter::~RollingShutter() {

}

// aslam::backend compatibility
void RollingShutter::update(const double * v) {
  _lineDelay += v[0];
}

int RollingShutter::minimalDimensions() const {
  return 1;
}

void RollingShutter::getParameters(Eigen::MatrixXd & P) const {
  P.resize(1, 1);
  P(0, 0) = _lineDelay;
}

void RollingShutter::setParameters(const Eigen::MatrixXd & P) {
  SM_ASSERT_GT(std::runtime_error, P.rows(), 0,
               "There must be at least one element in the matrix");
  SM_ASSERT_GT(std::runtime_error, P.cols(), 0,
               "There must be at least one element in the matrix");
  _lineDelay = P(0, 0);
}

Eigen::Vector2i RollingShutter::parameterSize() const {
  return Eigen::Vector2i(1, 1);
}

RollingShutter RollingShutter::getTestShutter() {
  return RollingShutter(10e-5);
}

}  // namespace cameras
}  // namespace aslam
