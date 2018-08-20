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
 *  @param[out] ecef the corresponding point in the geocentric ECEF frame.
 */
template <typename DerivedA, typename DerivedB>
void ecefPointFromLLH(const Eigen::MatrixBase<DerivedA> &llh,
                      Eigen::MatrixBase<DerivedB> const &ecef) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedB, 3);

    double latitude = llh(0), longitude = llh(1), height = llh(2);

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    double X, Y, Z;
    earth.Forward(latitude, longitude, height, X, Y, Z);

    const_cast<Eigen::MatrixBase<DerivedB> &>(ecef) << X, Y, Z;
};

/** Converts a point from LLH (Latitude [deg], Longitude [deg], Height[m]) to
 *  ECEF.
 *
 *  @param[in] ecef the input point in the geocentric ECEF frame.
 *  @param[out] llh the corresponding llh point as (Latitude, Longitude,
 *  Height). Height is relative to the WGS84 ellipsoid.
 */
template <typename DerivedA, typename DerivedB>
void llhPointFromECEF(const Eigen::MatrixBase<DerivedA> &ecef,
                      Eigen::MatrixBase<DerivedB> const &llh) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedB, 3);

    double X = ecef(0), Y = ecef(1), Z = ecef(2);

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    double latitude, longitude, height;
    earth.Reverse(X, Y, Z, latitude, longitude, height);

    const_cast<Eigen::MatrixBase<DerivedB> &>(llh) << latitude, longitude,
      height;
};

/** Computes the 3D Affine transform from a local datum-defined ENU frame to
 *  ECEF as a 4x4 row-major matrix.
 *
 *  @param[in] datum the LLH datum point defining the transform. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[out] T_ecef_enu the 4x4 row-major transformation matrix converting
 *  column-vector points from the local ENU frame defined by the datum point to
 *  ECEF.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 */
template <typename DerivedA, typename DerivedB>
void ecefFromENUTransformMatrix(const Eigen::MatrixBase<DerivedA> &datum,
                                Eigen::MatrixBase<DerivedB> const &T_ecef_enu,
                                bool datum_is_llh = true) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedB, 4, 4);

    // Both Forward() and Reverse() return the same rotation matrix from ENU
    // to ECEF
    std::vector<double> R_ecef_enu(9, 0.0);
    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
    double datum_X, datum_Y, datum_Z;

    if (datum_is_llh) {
        double latitude = datum(0), longitude = datum(1), height = datum(2);

        earth.Forward(
          latitude, longitude, height, datum_X, datum_Y, datum_Z, R_ecef_enu);
    } else {
        // Datum is already given in ECEF
        datum_X = datum(0);
        datum_Y = datum(1);
        datum_Z = datum(2);

        double latitude, longitude, height;
        earth.Reverse(
          datum_X, datum_Y, datum_Z, latitude, longitude, height, R_ecef_enu);
    }

    const_cast<Eigen::MatrixBase<DerivedB> &>(T_ecef_enu) << R_ecef_enu[0],
      R_ecef_enu[1], R_ecef_enu[2], datum_X, R_ecef_enu[3], R_ecef_enu[4],
      R_ecef_enu[5], datum_Y, R_ecef_enu[6], R_ecef_enu[7], R_ecef_enu[8],
      datum_Z, 0.0, 0.0, 0.0, 1.0;
};

/** Computes the 3D Affine transform from ECEF to a local datum-defined ENU
 *  frame as a 4x4 row-major matrix.
 *
 *  @param[in] datum the LLH datum point defining the transform. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[out] T_enu_ecef the 4x4 row-major transformation matrix converting
 *  column-vector points from ECEF to the local ENU frame defined by the datum
 *  point.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 */
template <typename DerivedA, typename DerivedB>
void enuFromECEFTransformMatrix(const Eigen::MatrixBase<DerivedA> &datum,
                                Eigen::MatrixBase<DerivedB> const &T_enu_ecef,
                                bool datum_is_llh = true) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedB, 4, 4);

    // Get T_ecef_enu and then invert it
    Eigen::Matrix4d T_ecef_enu;
    ecefFromENUTransformMatrix(datum, T_ecef_enu, datum_is_llh);

    // Affine inverse: [R | t]^(-1) = [ R^T | - R^T * t]
    (const_cast<Eigen::MatrixBase<DerivedB> &>(T_enu_ecef))
      .topLeftCorner(3, 3) = T_ecef_enu.topLeftCorner(3, 3).transpose();
    // Affine inverse translation component: -R_inverse * b
    //    with b as the 4th column of T_ecef_enu
    (const_cast<Eigen::MatrixBase<DerivedB> &>(T_enu_ecef))
      .topRightCorner(3, 1) = -T_ecef_enu.topLeftCorner(3, 3).transpose() *
                              T_ecef_enu.topRightCorner(3, 1);
    (const_cast<Eigen::MatrixBase<DerivedB> &>(T_enu_ecef)).row(3) =
      Eigen::MatrixXd::Zero(1, 4);
    (const_cast<Eigen::MatrixBase<DerivedB> &>(T_enu_ecef))(3, 3) = 1.0;
};

/** Converts a source point from LLH to a target ENU point in the local
 *  Cartesian ENU frame defined by the provided datum.
 *
 *  @param[in] point_llh the source LLH point to be converted to an ENU point.
 *  @param[in] enu_datum the LLH datum point defining the local ENU frame. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[out] point_enu the corresponding target point in the local ENU frame.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 */
template <typename DerivedA, typename DerivedB, typename DerivedC>
void enuPointFromLLH(const Eigen::MatrixBase<DerivedA> &point_llh,
                     const Eigen::MatrixBase<DerivedB> &enu_datum,
                     Eigen::MatrixBase<DerivedC> const &point_enu,
                     bool datum_is_llh = true) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedB, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedC, 3);

    Eigen::Vector3d enu_datum_llh;
    if (datum_is_llh) {
        enu_datum_llh(0) = enu_datum(0);
        enu_datum_llh(1) = enu_datum(1);
        enu_datum_llh(2) = enu_datum(2);
    } else {
        // Datum is ECEF
        llhPointFromECEF(enu_datum, enu_datum_llh);
    }

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::LocalCartesian localENU(
      enu_datum_llh(0), enu_datum_llh(1), enu_datum_llh(2), earth);

    localENU.Forward(point_llh(0),
                     point_llh(1),
                     point_llh(2),
                     (const_cast<Eigen::MatrixBase<DerivedC> &>(point_enu))(0),
                     (const_cast<Eigen::MatrixBase<DerivedC> &>(point_enu))(1),
                     (const_cast<Eigen::MatrixBase<DerivedC> &>(point_enu))(2));
};

/** Converts a source point from ENU in the local Cartesian ENU frame
 *  defined by the provided datum to a target LLH point.
 *
 *  @param[in] point_enu the source ENU point to be converted to an LLH point.
 *  @param[in] enu_datum the LLH datum point defining the local ENU frame. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[out] point_llh the corresponding target point in LLH.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 */
template <typename DerivedA, typename DerivedB, typename DerivedC>
void llhPointFromENU(const Eigen::MatrixBase<DerivedA> &point_enu,
                     const Eigen::MatrixBase<DerivedB> &enu_datum,
                     Eigen::MatrixBase<DerivedC> const &point_llh,
                     bool datum_is_llh = true) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedB, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedC, 3);
    Eigen::Vector3d enu_datum_llh;
    if (datum_is_llh) {
        enu_datum_llh(0) = enu_datum(0);
        enu_datum_llh(1) = enu_datum(1);
        enu_datum_llh(2) = enu_datum(2);
    } else {
        // Datum is ECEF
        llhPointFromECEF(enu_datum, enu_datum_llh);
    }

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::LocalCartesian localENU(
      enu_datum_llh(0), enu_datum_llh(1), enu_datum_llh(2), earth);

    localENU.Reverse(point_enu(0),
                     point_enu(1),
                     point_enu(2),
                     (const_cast<Eigen::MatrixBase<DerivedC> &>(point_llh))(0),
                     (const_cast<Eigen::MatrixBase<DerivedC> &>(point_llh))(1),
                     (const_cast<Eigen::MatrixBase<DerivedC> &>(point_llh))(2));
};

/** @} group geography */
}  // namespace wave
#endif  // WAVE_GEOGRAPHY_WORLD_FRAME_CONVERSIONS_HPP
