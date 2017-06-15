#ifndef ASLAM_GLOBAL_SHUTTER_HPP
#define ASLAM_GLOBAL_SHUTTER_HPP

#include <Eigen/Core>
#include "Duration.hpp"

namespace aslam {
namespace cameras {

class GlobalShutter {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
    DesignVariableDimension = 0
  };

  GlobalShutter();
  ~GlobalShutter();

  template<typename K>
  Duration temporalOffset(const K & /* k */ ) const {
    return Duration(0);
  }

  template<typename DERIVED_K, typename DERIVED_J>
  void temporalOffsetIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_K> & /* k */,
      const Eigen::MatrixBase<DERIVED_J> & outJ) const {
    Eigen::MatrixBase<DERIVED_J> & J =
        const_cast<Eigen::MatrixBase<DERIVED_J> &>(outJ);
    J.resize(0, 0);

  }

  // aslam::backend compatibility
  void update(const double * v);
  int minimalDimensions() const;
  void getParameters(Eigen::MatrixXd & P) const;
  void setParameters(const Eigen::MatrixXd & P);
  Eigen::Vector2i parameterSize() const;

  static GlobalShutter getTestShutter() {
    return GlobalShutter();
  }
};

}  // namespace cameras
}  // namespace aslam

#endif /* ASLAM_GLOBAL_SHUTTER_HPP */
