//#include <Eigen/Core>
//#include "wave/geography/world_frame_conversions.hpp"
//
//namespace wave {
//
//template<typename DerivedA, typename DerivedB>
//void ecefPointFromLLH(const Eigen::MatrixBase<DerivedA>& llh, Eigen::MatrixBase<DerivedB> const & ecef) {
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedB, 3);
//
//    double latitude = llh(0), longitude = llh(1), height = llh(2);
//
//    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
//
//    double X, Y, Z;
//    earth.Forward(latitude, longitude, height, X, Y, Z);
//
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(ecef))(0) = X;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(ecef))(1) = Y;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(ecef))(2) = Z;
//}
//
//template<typename DerivedA, typename DerivedB>
//void llhPointFromECEF(const Eigen::MatrixBase<DerivedA>& ecef, Eigen::MatrixBase<DerivedB> const & llh) {
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedB, 3);
//
//    double X = ecef(0), Y = ecef(1), Z = ecef(2);
//
//    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
//
//    double latitude, longitude, height;
//    earth.Reverse(X, Y, Z, latitude, longitude, height);
//
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(llh))(0) = latitude;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(llh))(1) = longitude;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(llh))(2) = height;
//}
//
//template<typename DerivedA, typename DerivedB>
//void ecefFromENUTransformMatrix(const Eigen::MatrixBase<DerivedA>& datum,
//                                Eigen::MatrixBase<DerivedB> const & T_ecef_enu,
//                                bool datum_is_llh) {
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
//    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedB, 4, 4);
//
//    // Both Forward() and Reverse() return the same rotation matrix from ENU
//    // to ECEF
//    std::vector<double> R_ecef_enu(9, 0.0);
//    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
//    double datum_X, datum_Y, datum_Z;
//
//    if (datum_is_llh) {
//        double latitude = datum(0), longitude = datum(1), height = datum(2);
//
//        earth.Forward(
//          latitude, longitude, height, datum_X, datum_Y, datum_Z, R_ecef_enu);
//    } else {
//        // Datum is already given in ECEF
//        datum_X = datum(0);
//        datum_Y = datum(1);
//        datum_Z = datum(2);
//
//        double latitude, longitude, height;
//        earth.Reverse(
//          datum_X, datum_Y, datum_Z, latitude, longitude, height, R_ecef_enu);
//    }
//
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(0,0) = R_ecef_enu[0];
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(0,1) = R_ecef_enu[1];
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(0,2) = R_ecef_enu[2];
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(0,3) = datum_X;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(1,0) = R_ecef_enu[3];
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(1,1) = R_ecef_enu[4];
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(1,2) = R_ecef_enu[5];
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(1,3) = datum_Y;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(2,0) = R_ecef_enu[6];
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(2,1) = R_ecef_enu[7];
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(2,2) = R_ecef_enu[8];
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(2,3) = datum_Z;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(3,0) = 0.0;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(3,1) = 0.0;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(3,2) = 0.0;
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_ecef_enu))(3,3) = 1.0;
//}
//
//template<typename DerivedA, typename DerivedB>
//void enuFromECEFTransformMatrix(const Eigen::MatrixBase<DerivedA>& datum,
//                                Eigen::MatrixBase<DerivedB> const & T_enu_ecef,
//                                bool datum_is_llh) {
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
//    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedB, 4, 4);
//
//    // Get T_ecef_enu and then invert it
//    Eigen::Matrix4d T_ecef_enu;
//    ecefFromENUTransformMatrix(datum, T_ecef_enu, datum_is_llh);
//
//    // Affine inverse: [R | t]^(-1) = [ R^T | - R^T * t]
//    // TODO(msmart/benskikos) - Move these functions to somewhere where we have
//    // matrix classes and ownership since it is ours. Manual transposition is
//    // undesirable. The below should be 2 lines:
//    // R_new = R.transpose(); t_new = - R.transpose()*t;
//    /*T_enu_ecef[0][0] = T_ecef_enu[0][0];
//    T_enu_ecef[0][1] = T_ecef_enu[1][0];
//    T_enu_ecef[0][2] = T_ecef_enu[2][0];
//    T_enu_ecef[1][0] = T_ecef_enu[0][1];
//    T_enu_ecef[1][1] = T_ecef_enu[1][1];
//    T_enu_ecef[1][2] = T_ecef_enu[2][1];
//    T_enu_ecef[2][0] = T_ecef_enu[0][2];
//    T_enu_ecef[2][1] = T_ecef_enu[1][2];
//    T_enu_ecef[2][2] = T_ecef_enu[2][2];
//
//    // Affine inverse translation component: -R_inverse * b
//    //    with b as the 4th column of T_ecef_enu
//    T_enu_ecef[0][3] = -T_enu_ecef[0][0] * T_ecef_enu[0][3]    //
//                       - T_enu_ecef[0][1] * T_ecef_enu[1][3]   //
//                       - T_enu_ecef[0][2] * T_ecef_enu[2][3];  //
//
//    T_enu_ecef[1][3] = -T_enu_ecef[1][0] * T_ecef_enu[0][3]    //
//                       - T_enu_ecef[1][1] * T_ecef_enu[1][3]   //
//                       - T_enu_ecef[1][2] * T_ecef_enu[2][3];  //
//
//    T_enu_ecef[2][3] = -T_enu_ecef[2][0] * T_ecef_enu[0][3]    //
//                       - T_enu_ecef[2][1] * T_ecef_enu[1][3]   //
//                       - T_enu_ecef[2][2] * T_ecef_enu[2][3];  //
//
//    // Last row is the same
//    T_enu_ecef[3][0] = 0.0;
//    T_enu_ecef[3][1] = 0.0;
//    T_enu_ecef[3][2] = 0.0;
//    T_enu_ecef[3][3] = 1.0; */
//
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_enu_ecef)).topLeftCorner(3,3) = T_ecef_enu.topLeftCorner(3,3).transpose();
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_enu_ecef)).col(4) = - T_ecef_enu.topLeftCorner(3,3).transpose() * T_ecef_enu.col(4);
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_enu_ecef)).row(4) = Eigen::MatrixXd::Zero(1,4);
//    (const_cast< Eigen::MatrixBase<DerivedB>& >(T_enu_ecef))(3)(3) = 1.0;
//}
//
//template<typename DerivedA, typename DerivedB, typename DerivedC>
//void enuPointFromLLH(const Eigen::MatrixBase<DerivedA>& point_llh,
//                     const Eigen::MatrixBase<DerivedB>& enu_datum,
//                     Eigen::MatrixBase<DerivedC> const & point_enu,
//                     bool datum_is_llh){
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedA, 3);
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedB, 3);
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedC, 3);
//
//    Eigen::Vector3d enu_datum_llh;
//    if (datum_is_llh) {
//        enu_datum_llh(0) = enu_datum(0);
//        enu_datum_llh(1) = enu_datum(1);
//        enu_datum_llh(2) = enu_datum(2);
//    } else {
//        // Datum is ECEF
//        llhPointFromECEF(enu_datum, enu_datum_llh);
//    }
//
//    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
//    GeographicLib::LocalCartesian localENU(
//      enu_datum_llh(0), enu_datum_llh(1), enu_datum_llh(2), earth);
//
//    localENU.Forward(point_llh(0),
//                     point_llh(1),
//                     point_llh(2),
//                     (const_cast< Eigen::MatrixBase<DerivedC>& >(point_enu))(0),
//                     (const_cast< Eigen::MatrixBase<DerivedC>& >(point_enu))(1),
//                     (const_cast< Eigen::MatrixBase<DerivedC>& >(point_enu))(2));
//}
//
//template<typename DerivedA, typename DerivedB, typename DerivedC>
//void llhPointFromENU(const Eigen::MatrixBase<DerivedA>& point_enu,
//                     const Eigen::MatrixBase<DerivedB>& enu_datum,
//                     Eigen::MatrixBase<DerivedC> const & point_llh,
//                     bool datum_is_llh) {
//    Eigen::Vector3d enu_datum_llh;
//    if (datum_is_llh) {
//        enu_datum_llh(0) = enu_datum(0);
//        enu_datum_llh(1) = enu_datum(1);
//        enu_datum_llh(2) = enu_datum(2);
//    } else {
//        // Datum is ECEF
//        llhPointFromECEF(enu_datum, enu_datum_llh);
//    }
//
//    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
//    GeographicLib::LocalCartesian localENU(
//      enu_datum_llh(0), enu_datum_llh(1), enu_datum_llh(2), earth);
//
//    localENU.Reverse(point_enu(0),
//                     point_enu(1),
//                     point_enu(2),
//                     (const_cast< Eigen::MatrixBase<DerivedC>& >(point_llh))(0),
//                     (const_cast< Eigen::MatrixBase<DerivedC>& >(point_llh))(1),
//                     (const_cast< Eigen::MatrixBase<DerivedC>& >(point_llh))(2));
//}
//
//}  // namespace wave
