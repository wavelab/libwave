#ifndef ASLAM_CAMERA_GEOMETRY_BASE_HPP
#define ASLAM_CAMERA_GEOMETRY_BASE_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

#include "aslam/cameras/Duration.hpp"

namespace sm {
namespace kinematics {
class Transformation;
}  // namespace kinematics
}  // namespace sm

namespace aslam {
// Forward declaration
class FrameBase;
namespace cameras {

// The base class for camera geometry...this will help exporting things to python or supporting
// runtime polymorphism but it will be a bit slower than pure templated code.
class CameraGeometryBase {
 public:
  typedef boost::shared_ptr<CameraGeometryBase> Ptr;
  typedef boost::shared_ptr<const CameraGeometryBase> ConstPtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraGeometryBase();
  virtual ~CameraGeometryBase();

  virtual size_t keypointDimension() const = 0;

  // The functions below take variable sized matrices and hence they are prefixed by "vs" to avoid confusion with
  // the fixed-size matrix versions available in derived classes. The default implementation of these "vs" functions
  // will be inefficient as it will call the fixed-size derived functions and copy the data into the variable size matrices.
  // Still, this base class is useful as it allows us to maintain a heterogeneous list of camera geometries and simplify the
  // python interface.
  virtual bool vsEuclideanToKeypoint(const Eigen::Vector3d & p,
                                     Eigen::VectorXd & outKeypoint) const = 0;
  virtual bool vsEuclideanToKeypoint(const Eigen::Vector3d & p,
                                     Eigen::VectorXd & outKeypoint,
                                     Eigen::MatrixXd & outJacobian) const = 0;

  virtual bool vsHomogeneousToKeypoint(const Eigen::Vector4d & ph,
                                       Eigen::VectorXd & outKeypoint) const = 0;

  virtual bool vsHomogeneousToKeypoint(const Eigen::Vector4d & ph,
                                       Eigen::VectorXd & outKeypoint,
                                       Eigen::MatrixXd & outJacobian) const = 0;

  // If the camera model is invertible, these functions produce 3d
  // points.  otherwise they produce points where the length has no
  // meaning. In that case the points are not necessarily on the
  // unit sphere in R^3 (that might be expensive to compute).
  virtual bool vsKeypointToEuclidean(const Eigen::VectorXd & keypoint,
                                     Eigen::Vector3d & outPoint) const = 0;

  virtual bool vsKeypointToEuclidean(const Eigen::VectorXd & keypoint,
                                     Eigen::VectorXd & outPoint,
                                     Eigen::MatrixXd &outJacobian) const = 0;

  virtual bool vsKeypointToHomogeneous(Eigen::VectorXd const & keypoint,
                                       Eigen::VectorXd & outPoint) const = 0;

  virtual bool vsKeypointToHomogeneous(Eigen::VectorXd const & keypoint,
                                       Eigen::VectorXd & outPoint,
                                       Eigen::MatrixXd & outJacobian) const = 0;

  // The amount of time elapsed between the start of the image and the
  // keypoint. For a global shutter camera, this can return Duration(0).
  virtual Duration vsTemporalOffset(const Eigen::VectorXd & keypoint) const = 0;

  // These functions are needed for unit testing.
  virtual Eigen::VectorXd createRandomKeypoint() const = 0;
  virtual Eigen::Vector3d createRandomVisiblePoint(double depth = -1) const = 0;

  virtual bool vsIsValid(const Eigen::VectorXd & keypoint) const = 0;
  virtual bool vsIsEuclideanVisible(const Eigen::Vector3d & p) const = 0;
  virtual bool vsIsHomogeneousVisible(const Eigen::Vector4d & ph) const = 0;
  virtual bool isProjectionInvertible() const = 0;

    /// \brief The width of the underlying image
  virtual int width() const = 0;

  /// \brief The height of the underlying image
  virtual int height() const = 0;

  // The amount of time elapsed between the start of the image and the
  // keypoint. For a global shutter camera, this can return Duration(0).
  virtual Duration temporalOffset(const Eigen::VectorXd & keypoint) const = 0;

  /// \brief print the internal parameters. This is just for debugging.
  virtual void print(std::ostream & out) = 0;

  // aslam::backend compatibility

  /// \brief the minimal dimensions of the projection parameters
  virtual int minimalDimensionsProjection() const = 0;

  /// \brief the minimal dimensions of the distortion parameters
  virtual int minimalDimensionsDistortion() const = 0;

  /// \brief the minimal dimensions of the shutter parameters
  virtual int minimalDimensionsShutter() const = 0;

  // aslam::backend compatibility

  /// \brief update the intrinsics
  virtual void update(const double * v, bool projection, bool distortion,
                      bool shutter) = 0;

  /// \brief Get the total number of dimensions of the intrinsic parameters
  virtual int minimalDimensions(bool projection, bool distortion,
                                bool shutter) const = 0;

  /// \brief get the intrinsic parameters.
  virtual void getParameters(Eigen::MatrixXd & P, bool projection,
                             bool distortion, bool shutter) const = 0;

  /// \brief set the intrinsic parameters.
  virtual void setParameters(const Eigen::MatrixXd & P, bool projection,
                             bool distortion, bool shutter) = 0;

  /// \brief return the Jacobian of the projection with respect to the intrinsics.
  virtual void euclideanToKeypointIntrinsicsJacobian(
      const Eigen::Vector3d & p, Eigen::MatrixXd & outJi,
      bool estimateProjection, bool estimateDistortion,
      bool estimateShutter) const = 0;

  /// \brief return the Jacobian of the projection with respect to the intrinsics.
  virtual void homogeneousToKeypointIntrinsicsJacobian(
      const Eigen::Vector4d & p, Eigen::MatrixXd & outJi,
      bool estimateProjection, bool estimateDistortion,
      bool estimateShutter) const = 0;

  /// \brief return the temporal offset with respect to the intrinsics.
  virtual void temporalOffsetIntrinsicsJacobian(
      const Eigen::VectorXd & keypoint, Eigen::MatrixXd & outJi,
      bool estimateProjection, bool estimateDistortion,
      bool estimateShutter) const = 0;

  virtual bool hasMask() const;

};

}  // namespace cameras
}  // namespace aslam

#endif /* ASLAM_CAMERA_GEOMETRY_BASE_HPP */
