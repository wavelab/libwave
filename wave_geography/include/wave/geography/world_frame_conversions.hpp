/** @file
 * @ingroup geography
 */

#ifndef WAVE_GEOGRAPHY_WORLD_FRAME_CONVERSIONS_HPP
#define WAVE_GEOGRAPHY_WORLD_FRAME_CONVERSIONS_HPP

#include <cmath>
#include <Eigen/Core>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace wave {
/** @addtogroup geography
 *  @{ */

/** Converts a point from LLH (Latitude [deg], Longitude [deg], Height[m]) to
 *  ECEF.
 *
 *  @param[in] llh the input llh point as (Latitude, Longitude, Height). Height
 *  is relative to the WGS84 ellipsoid.
 *  @return the corresponding point in the geocentric ECEF frame.
 */
template <typename Derived>
Eigen::Vector3d ecefPointFromLLH(const Eigen::MatrixBase<Derived> &llh) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    double latitude = llh(0), longitude = llh(1), height = llh(2);

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    double X, Y, Z;
    earth.Forward(latitude, longitude, height, X, Y, Z);

    return Eigen::Vector3d(X, Y, Z);
};

/** Converts a point from LLH (Latitude [deg], Longitude [deg], Height[m]) to
 *  ECEF.
 *
 *  @param[in] ecef the input point in the geocentric ECEF frame.
 *  @return the corresponding llh point as (Latitude, Longitude,
 *  Height). Height is relative to the WGS84 ellipsoid.
 */
template <typename Derived>
Eigen::Vector3d llhPointFromECEF(const Eigen::MatrixBase<Derived> &ecef) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    double X = ecef(0), Y = ecef(1), Z = ecef(2);

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    double latitude, longitude, height;
    earth.Reverse(X, Y, Z, latitude, longitude, height);

    return Eigen::Vector3d(latitude, longitude, height);
};

/** Computes the 3D Affine transform from a local datum-defined ENU frame to
 *  ECEF as a 4x4 row-major matrix.
 *
 *  @param[in] datum the ecef datum point defining the transform.
 *  @return the 4x4 row-major transformation matrix converting
 *  column-vector points from the local ENU frame defined by the datum point to
 *  ECEF.
 */
template <typename Derived>
Eigen::Matrix4d ecefEnuTransformFromEcef(
  const Eigen::MatrixBase<Derived> &datum) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    // Both Forward() and Reverse() return the same rotation matrix from ENU
    // to ECEF
    std::vector<double> R_ecef_enu(9, 0.0);
    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    double latitude, longitude, height;
    earth.Reverse(
      datum(0), datum(1), datum(2), latitude, longitude, height, R_ecef_enu);

    Eigen::Matrix4d transform;
    transform << R_ecef_enu[0], R_ecef_enu[1], R_ecef_enu[2], datum(0),
      R_ecef_enu[3], R_ecef_enu[4], R_ecef_enu[5], datum(1), R_ecef_enu[6],
      R_ecef_enu[7], R_ecef_enu[8], datum(2), 0.0, 0.0, 0.0, 1.0;
    return transform;
};

/** Computes the 3D Affine transform from ECEF to a local datum-defined ENU
 *  frame as a 4x4 row-major matrix.
 *
 *  @param[in] datum the ECEF datum point defining the transform.
 *  @return the 4x4 row-major transformation matrix converting
 *  column-vector points from ECEF to the local ENU frame defined by the datum
 *  point.
 */
template <typename Derived>
Eigen::Matrix4d enuEcefTransformFromEcef(
  const Eigen::MatrixBase<Derived> &datum) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    // Get T_ecef_enu and then invert it
    Eigen::Matrix4d T_ecef_enu = ecefEnuTransformFromEcef(datum);

    Eigen::Matrix4d T_enu_ecef;
    // Affine inverse: [R | t]^(-1) = [ R^T | - R^T * t]
    T_enu_ecef.topLeftCorner(3, 3) = T_ecef_enu.topLeftCorner(3, 3).transpose();
    // Affine inverse translation component: -R_inverse * b
    //    with b as the 4th column of T_ecef_enu
    T_enu_ecef.topRightCorner(3, 1) =
      -T_ecef_enu.topLeftCorner(3, 3).transpose() *
      T_ecef_enu.topRightCorner(3, 1);
    T_enu_ecef.row(3) = Eigen::MatrixXd::Zero(1, 4);
    T_enu_ecef(3, 3) = 1.0;

    return T_enu_ecef;
};

/** Converts a source point from LLH to a target ENU point in the local
 *  Cartesian ENU frame defined by the provided datum.
 *
 *  @param[in] point_llh the source LLH point to be converted to an ENU point.
 *  @param[in] enu_datum the LLH datum point defining the local ENU frame.
 *  @return the corresponding target point in the local ENU frame.
 */
template <typename DerivedA, typename DerivedB>
Eigen::Vector3d enuPointFromLLH(const Eigen::MatrixBase<DerivedA> &point_llh,
                                const Eigen::MatrixBase<DerivedB> &enu_datum) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedB, 3);

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::LocalCartesian localENU(
      enu_datum(0), enu_datum(1), enu_datum(2), earth);

    double A, B, C;
    localENU.Forward(point_llh(0), point_llh(1), point_llh(2), A, B, C);

    return Eigen::Vector3d(A, B, C);
};

/** Converts a source point from ENU in the local Cartesian ENU frame
 *  defined by the provided datum to a target LLH point.
 *
 *  @param[in] point_enu the source ENU point to be converted to an LLH point.
 *  @param[in] enu_datum the LLH datum point defining the local ENU frame.
 *  @return the corresponding target point in LLH.
 */
template <typename DerivedA, typename DerivedB>
Eigen::Vector3d llhPointFromENU(const Eigen::MatrixBase<DerivedA> &point_enu,
                                const Eigen::MatrixBase<DerivedB> &enu_datum) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedB, 3);

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::LocalCartesian localENU(
      enu_datum(0), enu_datum(1), enu_datum(2), earth);

    double A, B, C;
    localENU.Reverse(point_enu(0), point_enu(1), point_enu(2), A, B, C);

    return Eigen::Vector3d(A, B, C);
};

/** @} group geography */
}  // namespace wave
#endif  // WAVE_GEOGRAPHY_WORLD_FRAME_CONVERSIONS_HPP
