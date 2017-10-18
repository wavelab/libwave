#include <aslam/cameras/RadialTangentialDistortion.hpp>

namespace aslam {
namespace cameras {

RadialTangentialDistortion::RadialTangentialDistortion()
    : _k1(0),
      _k2(0),
      _p1(0),
      _p2(0) {

}

RadialTangentialDistortion::RadialTangentialDistortion(double k1, double k2,
                                                       double p1, double p2)
    : _k1(k1),
      _k2(k2),
      _p1(p1),
      _p2(p2) {

}

RadialTangentialDistortion::~RadialTangentialDistortion() {

}

// aslam::backend compatibility
void RadialTangentialDistortion::update(const double * v) {
  _k1 += v[0];
  _k2 += v[1];
  _p1 += v[2];
  _p2 += v[3];
}

int RadialTangentialDistortion::minimalDimensions() const {
  return IntrinsicsDimension;
}

void RadialTangentialDistortion::getParameters(Eigen::MatrixXd & S) const {
  S.resize(4, 1);
  S(0, 0) = _k1;
  S(1, 0) = _k2;
  S(2, 0) = _p1;
  S(3, 0) = _p2;
}

void RadialTangentialDistortion::setParameters(const Eigen::MatrixXd & S) {
  _k1 = S(0, 0);
  _k2 = S(1, 0);
  _p1 = S(2, 0);
  _p2 = S(3, 0);
}

Eigen::Vector2i RadialTangentialDistortion::parameterSize() const {
  return Eigen::Vector2i(4, 1);
}

RadialTangentialDistortion RadialTangentialDistortion::getTestDistortion() {
  return RadialTangentialDistortion(-0.2, 0.13, 0.0005, 0.0005);
}

}  // namespace cameras
}  // namespace aslam
