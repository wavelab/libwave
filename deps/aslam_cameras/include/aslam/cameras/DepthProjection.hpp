#ifndef ASLAM_CAMERAS_DEPTH_PROJECTION_HPP
#define ASLAM_CAMERAS_DEPTH_PROJECTION_HPP

#include "StaticAssert.hpp"

namespace aslam {
namespace cameras {

template<typename DISTORTION_T>
class DepthProjection {
 public:

  enum {
    KeypointDimension = 3
  };

  enum {
    IntrinsicsDimension = 4
  };
  enum {
    DesignVariableDimension = IntrinsicsDimension
  };

  typedef DISTORTION_T distortion_t;
  typedef Eigen::Matrix<double, KeypointDimension, 1> keypoint_t;
  typedef Eigen::Matrix<double, KeypointDimension, IntrinsicsDimension> jacobian_intrinsics_t;

  /// \brief Default constructor
  DepthProjection();

  DepthProjection(distortion_t distortion);

  DepthProjection(double focalLengthU, double focalLengthV, double imageCenterU,
                  double imageCenterV, int resolutionU, int resolutionV,
                  distortion_t distortion);

  DepthProjection(double focalLengthU, double focalLengthV, double imageCenterU,
                  double imageCenterV, int resolutionU, int resolutionV);

  /// \brief destructor.
  virtual ~DepthProjection();

  template<typename DERIVED_P, typename DERIVED_K>
  bool euclideanToKeypoint(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const;

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool euclideanToKeypoint(const Eigen::MatrixBase<DERIVED_P> & p,
                           const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
                           const Eigen::MatrixBase<DERIVED_JP> & outJp) const;

  template<typename DERIVED_P, typename DERIVED_K>
  bool homogeneousToKeypoint(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const;

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool homogeneousToKeypoint(const Eigen::MatrixBase<DERIVED_P> & p,
                             const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
                             const Eigen::MatrixBase<DERIVED_JP> & outJp) const;

  template<typename DERIVED_K, typename DERIVED_P>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                           const Eigen::MatrixBase<DERIVED_P> & outPoint) const;

  template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                           const Eigen::MatrixBase<DERIVED_P> & outPoint,
                           const Eigen::MatrixBase<DERIVED_JK> & outJk) const;

  template<typename DERIVED_K, typename DERIVED_P>
  bool keypointToHomogeneous(
      const Eigen::MatrixBase<DERIVED_K> & keypoint,
      const Eigen::MatrixBase<DERIVED_P> & outPoint) const;

  template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToHomogeneous(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                             const Eigen::MatrixBase<DERIVED_P> & outPoint,
                             const Eigen::MatrixBase<DERIVED_JK> & outJk) const;

  template<typename DERIVED_P, typename DERIVED_JI>
  void euclideanToKeypointIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JI> & outJi) const;

  template<typename DERIVED_P, typename DERIVED_JD>
  void euclideanToKeypointDistortionJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  template<typename DERIVED_P, typename DERIVED_JI>
  void homogeneousToKeypointIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JI> & outJi) const;

  template<typename DERIVED_P, typename DERIVED_JD>
  void homogeneousToKeypointDistortionJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  template<typename DERIVED_K>
  bool isValid(const Eigen::MatrixBase<DERIVED_K> & keypoint) const;

  template<typename DERIVED_P>
  bool isEuclideanVisible(const Eigen::MatrixBase<DERIVED_P> & p) const;

  template<typename DERIVED_P>
  bool isHomogeneousVisible(const Eigen::MatrixBase<DERIVED_P> & ph) const;

  // aslam::backend compatibility
  void update(const double * v);
  int minimalDimensions() const;
  void getParameters(Eigen::MatrixXd & P) const;
  void setParameters(const Eigen::MatrixXd & P);
  Eigen::Vector2i parameterSize() const;

  // \brief creates a random valid keypoint.
  virtual Eigen::VectorXd createRandomKeypoint() const;

  // \brief creates a random visible point. Negative depth means random between 0 and 100 meters.
  virtual Eigen::Vector3d createRandomVisiblePoint(double depth = -1.0) const;

  bool isProjectionInvertible() const {
    return false;
  }

  distortion_t & distortion() {
    return _distortion;
  }
  ;
  const distortion_t & distortion() const {
    return _distortion;
  }
  ;

  double focalLengthCol() const {
    return _fu;
  }
  double focalLengthRow() const {
    return _fv;
  }
  double opticalCenterCol() const {
    return _cu;
  }
  double opticalCenterRow() const {
    return _cv;
  }

  /// \brief The horizontal focal length in pixels.
  double fu() const {
    return _fu;
  }
  /// \brief The vertical focal length in pixels.
  double fv() const {
    return _fv;
  }
  /// \brief The horizontal image center in pixels.
  double cu() const {
    return _cu;
  }
  /// \brief The vertical image center in pixels.
  double cv() const {
    return _cv;
  }
  /// \brief The horizontal resolution in pixels.
  int ru() const {
    return _ru;
  }
  /// \brief The vertical resolution in pixels.
  int rv() const {
    return _rv;
  }

  /// \todo Fill in
  static DepthProjection<DISTORTION_T> getTestProjection() {
    return DepthProjection<DISTORTION_T>();
  }

 private:
  /// \brief The horizontal focal length in pixels.
  double _fu;
  /// \brief The vertical focal length in pixels.
  double _fv;
  /// \brief The horizontal image center in pixels.
  double _cu;
  /// \brief The vertical image center in pixels.
  double _cv;
  /// \brief The horizontal resolution in pixels.
  int _ru;
  /// \brief The vertical resolution in pixels.
  int _rv;

  /// \brief A computed value for speeding up computation.
  double _recip_fu;
  double _recip_fv;
  double _fu_over_fv;

  distortion_t _distortion;

};

}  // namespace cameras
}  // namespace aslam

#include "implementation/DepthProjection.hpp"

#endif /* ASLAM_CAMERAS_DEPTH_PROJECTION_HPP */
