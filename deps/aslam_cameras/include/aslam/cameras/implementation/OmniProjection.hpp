namespace aslam {

namespace cameras {

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T>::OmniProjection()
    : _xi(0.0),
      _fu(0.0),
      _fv(0.0),
      _cu(0.0),
      _cv(0.0),
      _ru(1),
      _rv(1) {
  updateTemporaries();

}

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T>::OmniProjection(double xi, double focalLengthU,
                                             double focalLengthV,
                                             double imageCenterU,
                                             double imageCenterV,
                                             int resolutionU, int resolutionV,
                                             distortion_t distortion)
    : _xi(xi),
      _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV),
      _distortion(distortion) {
  // 0
  updateTemporaries();

}

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T>::OmniProjection(double xi, double focalLengthU,
                                             double focalLengthV,
                                             double imageCenterU,
                                             double imageCenterV,
                                             int resolutionU, int resolutionV)
    : _xi(xi),
      _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV) {
  updateTemporaries();
}

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T>::~OmniProjection() {
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool OmniProjection<DISTORTION_T>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_K> & outKeypointConst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  Eigen::MatrixBase<DERIVED_K> & outKeypoint = const_cast<Eigen::MatrixBase<
      DERIVED_K> &>(outKeypointConst);
  outKeypoint.derived().resize(2);
  //    SM_OUT(p.transpose());
  double d = p.norm();

  // Check if point will lead to a valid projection
  if (p[2] <= -(_fov_parameter * d))
    return false;

  double rz = 1.0 / (p[2] + _xi * d);
  outKeypoint[0] = p[0] * rz;
  outKeypoint[1] = p[1] * rz;
  //std::cout << "normalize\n";
  //SM_OUT(d);
  //SM_OUT(rz);
  //SM_OUT(outKeypoint[0]);
  //SM_OUT(outKeypoint[1]);

  _distortion.distort(outKeypoint);
  //std::cout << "distort\n";
  //SM_OUT(outKeypoint[0]);
  //SM_OUT(outKeypoint[1]);

  outKeypoint[0] = _fu * outKeypoint[0] + _cu;
  outKeypoint[1] = _fv * outKeypoint[1] + _cv;
  //std::cout << "project\n";
  //SM_OUT(outKeypoint[0]);
  //SM_OUT(outKeypoint[1]);

  // Check if keypoint lies on the sensor
  return isValid(outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool OmniProjection<DISTORTION_T>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_K> & outKeypointConst,
    const Eigen::MatrixBase<DERIVED_JP> & outJp) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JP>, 2, 3);

  Eigen::MatrixBase<DERIVED_K> & outKeypoint = const_cast<Eigen::MatrixBase<
      DERIVED_K> &>(outKeypointConst);
  outKeypoint.derived().resize(2);

  // Jacobian:
  Eigen::MatrixBase<DERIVED_JP> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp);
  J.derived().resize(KeypointDimension, 3);
  J.setZero();

  double d = p.norm();

  // Check if point will lead to a valid projection
  if (p[2] <= -(_fov_parameter * d))
    return false;

  // project the point
  double rz = 1.0 / (p[2] + _xi * d);
  outKeypoint[0] = p[0] * rz;
  outKeypoint[1] = p[1] * rz;

  // Calculate jacobian
  rz = rz * rz / d;
  J(0, 0) = rz * (d * p[2] + _xi * (p[1] * p[1] + p[2] * p[2]));
  J(1, 0) = -rz * _xi * p[0] * p[1];
  J(0, 1) = J(1, 0);
  J(1, 1) = rz * (d * p[2] + _xi * (p[0] * p[0] + p[2] * p[2]));
  rz = rz * (-_xi * p[2] - d);
  J(0, 2) = p[0] * rz;
  J(1, 2) = p[1] * rz;

  Eigen::Matrix2d Jd;
  _distortion.distort(outKeypoint, Jd);

  rz = _fu * (J(0, 0) * Jd(0, 0) + J(1, 0) * Jd(0, 1));
  J(1, 0) = _fv * (J(0, 0) * Jd(1, 0) + J(1, 0) * Jd(1, 1));
  J(0, 0) = rz;

  rz = _fu * (J(0, 1) * Jd(0, 0) + J(1, 1) * Jd(0, 1));
  J(1, 1) = _fv * (J(0, 1) * Jd(1, 0) + J(1, 1) * Jd(1, 1));
  J(0, 1) = rz;

  rz = _fu * (J(0, 2) * Jd(0, 0) + J(1, 2) * Jd(0, 1));
  J(1, 2) = _fv * (J(0, 2) * Jd(1, 0) + J(1, 2) * Jd(1, 1));
  J(0, 2) = rz;

  outKeypoint[0] = _fu * outKeypoint[0] + _cu;
  outKeypoint[1] = _fv * outKeypoint[1] + _cv;

  return isValid(outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool OmniProjection<DISTORTION_T>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & ph,
    const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  // hope this works... (required to have valid static asserts)
  if (ph[3] < 0)
    return euclideanToKeypoint(-ph.derived().template head<3>(), outKeypoint);
  else
    return euclideanToKeypoint(ph.derived().template head<3>(), outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool OmniProjection<DISTORTION_T>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & ph,
    const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
    const Eigen::MatrixBase<DERIVED_JP> & outJp) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JP>, 2, 4);

  Eigen::MatrixBase<DERIVED_JP> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp);
  J.derived().resize(KeypointDimension, 4);
  J.setZero();

  if (ph[3] < 0) {
    bool success = euclideanToKeypoint(
        -ph.derived().template head<3>(), outKeypoint,
        J.derived().template topLeftCorner<2, 3>());
    J = -J;
    return success;
  } else {
    return euclideanToKeypoint(ph.derived().template head<3>(), outKeypoint,
                               J.derived().template topLeftCorner<2, 3>());
  }
}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P>
bool OmniProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPointConst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  Eigen::MatrixBase<DERIVED_P> & outPoint = const_cast<Eigen::MatrixBase<
      DERIVED_P> &>(outPointConst);
  outPoint.derived().resize(3);

  // Unproject...
  outPoint[0] = _recip_fu * (keypoint[0] - _cu);
  outPoint[1] = _recip_fv * (keypoint[1] - _cv);

  // Re-distort
  _distortion.undistort(outPoint.derived().template head<2>());

  double rho2_d = outPoint[0] * outPoint[0] + outPoint[1] * outPoint[1];

  if (!isUndistortedKeypointValid(rho2_d))
    return false;

  outPoint[2] = 1
      - _xi * (rho2_d + 1) / (_xi + sqrt(1 + (1 - _xi * _xi) * rho2_d));

  return true;

}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool OmniProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPointConst,
    const Eigen::MatrixBase<DERIVED_JK> & outJk) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JK>, 3, 2);

  Eigen::MatrixBase<DERIVED_P> & outPoint = const_cast<Eigen::MatrixBase<
      DERIVED_P> &>(outPointConst);
  outPoint.derived().resize(3);

  // Unproject...
  outPoint[0] = _recip_fu * (keypoint[0] - _cu);
  outPoint[1] = _recip_fv * (keypoint[1] - _cv);

  // Re-distort
  Eigen::MatrixXd Jd(2, 2);
  _distortion.undistort(outPoint.derived().template head<2>(), Jd);

  double rho2_d = outPoint[0] * outPoint[0] + outPoint[1] * outPoint[1];

  if (!isUndistortedKeypointValid(rho2_d))
    return false;

  double tmpZ = sqrt(-(rho2_d) * (_xi * _xi - 1.0) + 1.0);
  double tmpA = _xi + tmpZ;
  double recip_tmpA = 1.0 / tmpA;

  outPoint[2] = 1 - _xi * (rho2_d + 1) * recip_tmpA;

  // \todo analytical Jacobian
  Eigen::MatrixBase<DERIVED_JK> & mbJk =
      const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk);
  DERIVED_JK & Jk = mbJk.derived();

  double r0 = outPoint[0];
  double r1 = outPoint[1];

  double recip_tmpA2 = recip_tmpA * recip_tmpA;
  //double recip_tmpA2 = 1.0/tmpA2;
  double tmpB = 1.0 / tmpZ;
  Jk(0, 0) = Jd(0, 0) * _recip_fu;
  Jk(0, 1) = Jd(0, 1) * _recip_fv;
  Jk(1, 0) = Jd(1, 0) * _recip_fu;
  Jk(1, 1) = Jd(1, 1) * _recip_fv;

  double mul = -_xi
      * (2.0 * recip_tmpA
          + recip_tmpA2 * (_xi * _xi - 1.0) * tmpB * (rho2_d + 1.0));
  Eigen::Vector2d J3;
  J3[0] = r0 * mul;
  J3[1] = r1 * mul;

  Jk.row(2) = J3.transpose() * Jd
      * Eigen::Vector2d(_recip_fu, _recip_fv).asDiagonal();

  //ASLAM_CAMERAS_ESTIMATE_JACOBIAN(this,keypointToEuclidean, keypoint, 1e-5, Jk);

  return true;

}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P>
bool OmniProjection<DISTORTION_T>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPoint) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  Eigen::MatrixBase<DERIVED_P> & p =
      const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPoint);
  p.derived().resize(4);
  p[3] = 0.0;
  return keypointToEuclidean(keypoint, p.derived().template head<3>());

}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool OmniProjection<DISTORTION_T>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPoint,
    const Eigen::MatrixBase<DERIVED_JK> & outJk) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JK>, 4, 2);

  Eigen::MatrixBase<DERIVED_JK> & Jk =
      const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk);
  Jk.derived().resize(4, 2);
  Jk.setZero();

  Eigen::MatrixBase<DERIVED_P> & p =
      const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPoint);
  p.derived().resize(4);
  p[3] = 0.0;

  return keypointToEuclidean(keypoint, p.template head<3>(),
                             Jk.template topLeftCorner<3, 2>());

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JI>
void OmniProjection<DISTORTION_T>::euclideanToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JI> & outJi) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JI>, (int) KeypointDimension, 5);

  Eigen::MatrixBase<DERIVED_JI> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JI> &>(outJi);
  J.derived().resize(KeypointDimension, 5);
  J.setZero();

  keypoint_t kp;
  double d = p.norm();
  double rz = 1.0 / (p[2] + _xi * d);
  kp[0] = p[0] * rz;
  kp[1] = p[1] * rz;

  Eigen::Vector2d Jxi;
  Jxi[0] = -kp[0] * d * rz;
  Jxi[1] = -kp[1] * d * rz;

  Eigen::Matrix2d Jd;
  _distortion.distort(kp, Jd);

  Jd.row(0) *= _fu;
  Jd.row(1) *= _fv;
  J.col(0) = Jd * Jxi;

  J(0, 1) = kp[0];
  J(0, 3) = 1;

  J(1, 2) = kp[1];
  J(1, 4) = 1;

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JD>
void OmniProjection<DISTORTION_T>::euclideanToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);

  double d = p.norm();
  double rz = 1.0 / (p[2] + _xi * d);
  keypoint_t kp;
  kp[0] = p[0] * rz;
  kp[1] = p[1] * rz;

  _distortion.distortParameterJacobian(kp, outJd);

  Eigen::MatrixBase<DERIVED_JD> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);

  J.row(0) *= _fu;
  J.row(1) *= _fv;

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JI>
void OmniProjection<DISTORTION_T>::homogeneousToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JI> & outJi) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);

  if (p[3] < 0.0) {
    euclideanToKeypointIntrinsicsJacobian(-p.derived().template head<3>(),
                                          outJi);
  } else {
    euclideanToKeypointIntrinsicsJacobian(p.derived().template head<3>(),
                                          outJi);
  }

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JD>
void OmniProjection<DISTORTION_T>::homogeneousToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);

  if (p[3] < 0.0) {
    euclideanToKeypointDistortionJacobian(-p.derived().template head<3>(),
                                          outJd);
  } else {
    euclideanToKeypointDistortionJacobian(p.derived().template head<3>(),
                                          outJd);
  }

}

// \brief creates a random valid keypoint.
template<typename DISTORTION_T>
Eigen::VectorXd OmniProjection<DISTORTION_T>::createRandomKeypoint() const {

  // This is tricky...The camera model defines a circle on the normalized image
  // plane and the projection equations don't work outside of it.
  // With some manipulation, we can see that, on the normalized image plane,
  // the edge of this circle is at u^2 + v^2 = 1/(xi^2 - 1)
  // So: this function creates keypoints inside this boundary.

  // Create a point on the normalized image plane inside the boundary.
  // This is not efficient, but it should be correct.

  Eigen::Vector2d u(_ru + 1, _rv + 1);

  while (u[0] <= 0 || u[0] >= _ru - 1 || u[1] <= 0 || u[1] >= _rv - 1) {
    u.setRandom();
    u = u - Eigen::Vector2d(0.5, 0.5);
    u /= u.norm();
    u *= ((double) rand() / (double) RAND_MAX) * _one_over_xixi_m_1;

    // Now we run the point through distortion and projection.
    // Apply distortion
    _distortion.distort(u);

    u[0] = _fu * u[0] + _cu;
    u[1] = _fv * u[1] + _cv;
  }

  return u;
}

// \brief creates a random visible point. Negative depth means random between 0 and 100 meters.
template<typename DISTORTION_T>
Eigen::Vector3d OmniProjection<DISTORTION_T>::createRandomVisiblePoint(
    double depth) const {
  Eigen::VectorXd y = createRandomKeypoint();
  Eigen::Vector3d p;
  keypointToEuclidean(y, p);

  if (depth < 0.0) {
    depth = ((double) rand() / (double) RAND_MAX) * 100.0;
  }

  p /= p.norm();

  // Muck with the depth. This doesn't change the pointing direction.
  p *= depth;
  return p;

}

template<typename DISTORTION_T>
template<typename DERIVED_K>
bool OmniProjection<DISTORTION_T>::isValid(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  return keypoint(0) >= 0 && keypoint(0) < ru() && keypoint(1) >= 0
      && keypoint(1) < rv();
}

template<typename DISTORTION_T>
bool OmniProjection<DISTORTION_T>::isUndistortedKeypointValid(
    const double rho2_d) const {
  return xi() <= 1.0 || rho2_d <= _one_over_xixi_m_1;
}

template<typename DISTORTION_T>
template<typename DERIVED_K>
bool OmniProjection<DISTORTION_T>::isLiftable(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  // Unproject...
  Eigen::Vector2d y;
  y[0] = _recip_fu * (keypoint[0] - _cu);
  y[1] = _recip_fv * (keypoint[1] - _cv);

  // Re-distort
  _distortion.undistort(y);

  // Now check if it is on the sensor
  double rho2_d = y[0] * y[0] + y[1] * y[1];
  return isUndistortedKeypointValid(rho2_d);
}

template<typename DISTORTION_T>
template<typename DERIVED_P>
bool OmniProjection<DISTORTION_T>::isEuclideanVisible(
    const Eigen::MatrixBase<DERIVED_P> & p) const {
  keypoint_t k;
  return euclideanToKeypoint(p, k);

}

template<typename DISTORTION_T>
template<typename DERIVED_P>
bool OmniProjection<DISTORTION_T>::isHomogeneousVisible(
    const Eigen::MatrixBase<DERIVED_P> & ph) const {
  keypoint_t k;
  return homogeneousToKeypoint(ph, k);
}

template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::updateTemporaries() {
  _recip_fu = 1.0 / _fu;
  _recip_fv = 1.0 / _fv;
  _fu_over_fv = _fu / _fv;
  _one_over_xixi_m_1 = 1.0 / (_xi * _xi - 1);
  _fov_parameter = (_xi <= 1.0) ? _xi : 1 / _xi;
}

// aslam::backend compatibility
template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::update(const double * v) {
  _xi += v[0];
  _fu += v[1];
  _fv += v[2];
  _cu += v[3];
  _cv += v[4];

  updateTemporaries();

}
template<typename DISTORTION_T>
int OmniProjection<DISTORTION_T>::minimalDimensions() const {
  return 5;
}

template<typename DISTORTION_T>
Eigen::Vector2i OmniProjection<DISTORTION_T>::parameterSize() const {
  return Eigen::Vector2i(5, 1);
}

template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::getParameters(Eigen::MatrixXd & P) const {
  P.resize(5, 1);
  P << _xi, _fu, _fv, _cu, _cv;
}
template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::setParameters(const Eigen::MatrixXd & P) {
  SM_ASSERT_EQ(std::runtime_error, P.rows(), 5, "Incorrect size");
  SM_ASSERT_EQ(std::runtime_error, P.cols(), 1, "Incorrect size");
  _xi = P(0, 0);
  _fu = P(1, 0);
  _fv = P(2, 0);
  _cu = P(3, 0);
  _cv = P(4, 0);

  updateTemporaries();
}

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T> OmniProjection<DISTORTION_T>::getTestProjection() {
  return OmniProjection<DISTORTION_T>(0.9, 400, 400, 320, 240, 640, 480,
                                      DISTORTION_T::getTestDistortion());
}

template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::resizeIntrinsics(double scale) {
  _fu *= scale;
  _fv *= scale;
  _cu *= scale;
  _cv *= scale;
  _ru = _ru * scale;
  _rv = _rv * scale;

  updateTemporaries();
}

namespace detail {

inline double square(double x) {
  return x * x;
}
inline float square(float x) {
  return x * x;
}
inline double hypot(double a, double b) {
  return sqrt(square(a) + square(b));
}

}  // namespace detail

}  // namespace cameras
}  // namespace aslam
