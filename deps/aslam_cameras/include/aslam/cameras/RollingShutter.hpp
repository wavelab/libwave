#ifndef ASLAM_ROLLING_SHUTTER_HPP
#define ASLAM_ROLLING_SHUTTER_HPP

#include <Eigen/Core>
#include "Duration.hpp"


namespace aslam {
namespace cameras {

class RollingShutter {

 public:

  enum {
    DesignVariableDimension = 0
  };

  RollingShutter();
  RollingShutter(double lineDelay);
  ~RollingShutter();

  template<typename DERIVED_K>
  Duration temporalOffset(const Eigen::MatrixBase<DERIVED_K> & k) const {
    return Duration(_lineDelay * k[1]);
  }

  // aslam::backend compatibility
  void update(const double * v);
  int minimalDimensions() const;
  void getParameters(Eigen::MatrixXd & P) const;
  void setParameters(const Eigen::MatrixXd & P);
  Eigen::Vector2i parameterSize() const;

  static RollingShutter getTestShutter();

  double lineDelay() const {
    return _lineDelay;
  }
  ;

  /// The resulting jacobian assumes the output of "temporal offset" is in seconds.
  template<typename DERIVED_K, typename DERIVED_J>
  void temporalOffsetIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_K> & k,
      const Eigen::MatrixBase<DERIVED_J> & outJ) const {
    Eigen::MatrixBase<DERIVED_J> & J =
        const_cast<Eigen::MatrixBase<DERIVED_J> &>(outJ);
    J.resize(1, 1);
    J(0, 0) = k[1];
  }

 private:
  // RS camera time between start of integration of two consecutive lines in seconds
  double _lineDelay;

};

}  // namespace cameras
}  // namespace aslam

#endif /* ASLAM_ROLLING_SHUTTER_HPP */
