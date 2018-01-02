#include "wave/odometry/weighting_functions.hpp"
#include <unsupported/Eigen/MatrixFunctions>  // For matrix square root

namespace {

Eigen::Matrix3d calculatePointToLineJpt(const double *const ptA, const double *const ptB) {
    auto t2 = ptA[0] - ptB[0];
    auto t3 = t2 * t2;
    auto t4 = ptA[1] - ptB[1];
    auto t5 = ptA[2] - ptB[2];
    auto t6 = t4 * t4;
    auto t7 = t5 * t5;
    auto t8 = t3 + t6 + t7;
    auto t9 = 1.0 / t8;

    Eigen::Matrix3d Jr_p1;
    Jr_p1 << -t3 * t9 + 1.0, -t2 * t4 * t9, -t2 * t5 * t9, -t2 * t4 * t9, -t6 * t9 + 1.0, -t4 * t5 * t9, -t2 * t5 * t9,
      -t4 * t5 * t9, -t7 * t9 + 1.0;
    return Jr_p1;
}

Eigen::Matrix3d calculatePointToLineJpA(const double *const pt, const double *const ptA, const double *const ptB) {
    auto t2 = ptA[0] - ptB[0];
    auto t3 = ptA[1] - ptB[1];
    auto t4 = ptA[2] - ptB[2];
    auto t5 = t2 * t2;
    auto t6 = t3 * t3;
    auto t7 = t4 * t4;
    auto t8 = t5 + t6 + t7;
    auto t9 = 1.0 / t8;
    auto t10 = pt[0] - ptA[0];
    auto t11 = t2 * t10;
    auto t12 = pt[1] - ptA[1];
    auto t13 = t3 * t12;
    auto t14 = pt[2] - ptA[2];
    auto t15 = t4 * t14;
    auto t16 = t11 + t13 + t15;
    auto t17 = 1.0 / (t8 * t8);
    auto t18 = ptA[0] * 2.0;
    auto t19 = ptB[0] * 2.0;
    auto t20 = t18 - t19;
    auto t21 = ptA[1] * 2.0;
    auto t22 = ptB[1] * 2.0;
    auto t23 = t21 - t22;
    auto t24 = ptA[2] * 2.0;
    auto t25 = ptB[2] * 2.0;
    auto t26 = t24 - t25;
    auto t27 = pt[0] + ptB[0] - t18;
    auto t28 = pt[1] + ptB[1] - t21;
    auto t29 = pt[2] + ptB[2] - t24;

    Eigen::Matrix3d Jr_pA;
    Jr_pA << -t9 * t16 - t2 * t9 * (pt[0] - ptA[0] * 2.0 + ptB[0]) + t2 * t16 * t17 * t20 - 1.0,
      -t3 * t9 * t27 + t3 * t16 * t17 * t20, -t4 * t9 * t27 + t4 * t16 * t17 * t20,
      -t2 * t9 * (pt[1] - ptA[1] * 2.0 + ptB[1]) + t2 * t16 * t17 * t23,
      -t9 * t16 - t3 * t9 * t28 + t3 * t16 * t17 * t23 - 1.0, -t4 * t9 * t28 + t4 * t16 * t17 * t23,
      -t2 * t9 * (pt[2] - ptA[2] * 2.0 + ptB[2]) + t2 * t16 * t17 * t26, -t3 * t9 * t29 + t3 * t16 * t17 * t26,
      -t9 * t16 - t4 * t9 * t29 + t4 * t16 * t17 * t26 - 1.0;
    return Jr_pA;
}

Eigen::Matrix3d calculatePointToLineJpB(const double *const pt, const double *const ptA, const double *const ptB) {
    auto t2 = ptA[0] - ptB[0];
    auto t3 = ptA[1] - ptB[1];
    auto t4 = ptA[2] - ptB[2];
    auto t5 = pt[0] - ptA[0];
    auto t6 = t2 * t2;
    auto t7 = t3 * t3;
    auto t8 = t4 * t4;
    auto t9 = t6 + t7 + t8;
    auto t10 = 1.0 / t9;
    auto t11 = t2 * t5;
    auto t12 = pt[1] - ptA[1];
    auto t13 = t3 * t12;
    auto t14 = pt[1] - ptA[2];
    auto t15 = t4 * t14;
    auto t16 = t11 + t13 + t15;
    auto t17 = 1.0 / (t9 * t9);
    auto t18 = ptA[0] * 2.0;
    auto t19 = ptB[0] * 2.0;
    auto t20 = t18 - t19;
    auto t21 = t10 * t16;
    auto t22 = ptA[1] * 2.0;
    auto t23 = ptB[1] * 2.0;
    auto t24 = t22 - t23;
    auto t25 = ptA[2] * 2.0;
    auto t26 = ptB[2] * 2.0;
    auto t27 = t25 - t26;

    Eigen::Matrix3d Jr_pB;
    Jr_pB << t21 + t2 * t5 * t10 - t2 * t16 * t17 * t20, t3 * t5 * t10 - t3 * t16 * t17 * t20,
      t4 * t5 * t10 - t4 * t16 * t17 * t20, t2 * t10 * t12 - t2 * t16 * t17 * t24,
      t21 + t3 * t10 * t12 - t3 * t16 * t17 * t24, t4 * t10 * t12 - t4 * t16 * t17 * t24,
      t2 * t10 * t14 - t2 * t16 * t17 * t27, t3 * t10 * t14 - t3 * t16 * t17 * t27,
      t21 + t4 * t10 * t14 - t4 * t16 * t17 * t27;
    return Jr_pB;
}

Eigen::Matrix<double, 1, 3> calculatePointToPlaneJpt(const double *const pA,
                                                     const double *const pB,
                                                     const double *const pC) {
    auto t2 = pB[1] - pC[1];
    auto t3 = pA[1] - pB[1];
    auto t5 = pA[0] - pB[0];
    auto t8 = pB[0] - pC[0];
    auto t16 = t2 * t5;
    auto t17 = t3 * t8;
    auto t18 = t16 - t17;
    auto t4 = std::abs(t18);
    auto t6 = pB[2] - pC[2];
    auto t7 = pA[2] - pB[2];
    auto t13 = t5 * t6;
    auto t14 = t7 * t8;
    auto t15 = t13 - t14;
    auto t9 = std::abs(t15);
    auto t10 = t3 * t6;
    auto t21 = t2 * t7;
    auto t11 = t10 - t21;
    auto t12 = std::abs(t11);
    auto t19 = t4 * t4;
    auto t20 = t9 * t9;
    auto t22 = t12 * t12;
    auto t23 = t19 + t20 + t22;
    auto t24 = 1.0 / sqrt(t23);

    Eigen::Matrix<double, 1, 3> retval;
    retval << -t11 * t24, t15 * t24, -t18 * t24;
    return retval;
}

Eigen::Matrix<double, 1, 3> calculatePointToPlaneJpA(const double *const pt,
                                                     const double *const pA,
                                                     const double *const pB,
                                                     const double *const pC) {
    auto t3 = pA[0] - pB[0];
    auto t5 = pB[0] - pC[0];
    auto t7 = pA[1] - pB[1];
    auto t9 = pB[1] - pC[1];
    auto t11 = t3 * t9;
    auto t12 = t5 * t7;
    auto t13 = t11 - t12;
    auto t2 = std::abs(t13);
    auto t4 = pB[2] - pC[2];
    auto t8 = pA[2] - pB[2];
    auto t15 = t3 * t4;
    auto t16 = t5 * t8;
    auto t17 = t15 - t16;
    auto t6 = std::abs(t17);
    auto t19 = t4 * t7;
    auto t20 = t8 * t9;
    auto t21 = t19 - t20;
    auto t10 = std::abs(t21);
    auto t14 = t2 * t2;
    auto t18 = t6 * t6;
    auto t22 = t10 * t10;
    auto t23 = t14 + t18 + t22;
    auto t24 = 1.0 / std::sqrt(t23);
    auto t25 = pt[2] - pC[2];
    auto t26 = t13 > 0 ? 1 : -1;
    auto t27 = t2 * t9 * t26 * 2.0;
    auto t28 = t17 > 0 ? 1 : -1;
    auto t29 = t4 * t6 * t28 * 2.0;
    auto t30 = t27 + t29;
    auto t31 = pt[1] - pC[1];
    auto t32 = 1.0 / std::pow(t23, 1.5);
    auto t33 = pt[0] - pC[0];
    auto t34 = t21 > 0 ? 1 : -1;
    auto t36 = t2 * t5 * t26 * 2.0;
    auto t37 = t4 * t10 * t34 * 2.0;
    auto t35 = t36 - t37;
    auto t38 = t5 * t6 * t28 * 2.0;
    auto t39 = t9 * t10 * t34 * 2.0;
    auto t40 = t38 + t39;

    Eigen::Matrix<double, 1, 3> retval;
    retval << -t9 * t24 * t25 + t4 * t24 * t31 + t13 * t25 * t30 * t32 * (1.0 / 2.0) -
                t17 * t30 * t31 * t32 * (1.0 / 2.0) + t21 * t30 * t32 * t33 * (1.0 / 2.0),
      t5 * t24 * t25 - t4 * t24 * t33 - t13 * t25 * t32 * t35 * (1.0 / 2.0) + t17 * t31 * t32 * t35 * (1.0 / 2.0) -
        t21 * t32 * t33 * t35 * (1.0 / 2.0),
      -t5 * t24 * t31 + t9 * t24 * t33 - t13 * t25 * t32 * t40 * (1.0 / 2.0) + t17 * t31 * t32 * t40 * (1.0 / 2.0) -
        t21 * t32 * t33 * t40 * (1.0 / 2.0);
    return retval;
}

Eigen::Matrix<double, 1, 3> calculatePointToPlaneJpB(const double *const pt,
                                                     const double *const pA,
                                                     const double *const pB,
                                                     const double *const pC) {
    auto t3 = pA[0] - pB[0];
    auto t4 = pB[0] - pC[0];
    auto t6 = pA[1] - pB[1];
    auto t9 = pB[1] - pC[1];
    auto t11 = t3 * t9;
    auto t12 = t4 * t6;
    auto t13 = t11 - t12;
    auto t2 = std::abs(t13);
    auto t7 = pB[2] - pC[2];
    auto t8 = pA[2] - pB[2];
    auto t15 = t3 * t7;
    auto t16 = t4 * t8;
    auto t17 = t15 - t16;
    auto t5 = std::abs(t17);
    auto t19 = t6 * t7;
    auto t20 = t8 * t9;
    auto t21 = t19 - t20;
    auto t10 = std::abs(t21);
    auto t14 = t2 * t2;
    auto t18 = t5 * t5;
    auto t22 = t10 * t10;
    auto t23 = t14 + t18 + t22;
    auto t24 = 1.0 / std::sqrt(t23);
    auto t25 = pA[1] - pC[1];
    auto t26 = pA[2] - pC[2];
    auto t27 = pt[2] - pC[2];
    auto t28 = t13 > 0 ? 1 : -1;
    auto t29 = t2 * t25 * t28 * 2.0;
    auto t30 = t17 > 0 ? 1 : -1;
    auto t31 = t5 * t26 * t30 * 2.0;
    auto t32 = t29 + t31;
    auto t33 = pt[1] - pC[1];
    auto t34 = 1.0 / std::pow(t23, 1.5);
    auto t35 = pt[0] - pC[0];
    auto t36 = pA[0] - pC[0];
    auto t37 = t21 > 0 ? 1 : -1;
    auto t39 = t2 * t28 * t36 * 2.0;
    auto t40 = t10 * t26 * t37 * 2.0;
    auto t38 = t39 - t40;
    auto t41 = t5 * t30 * t36 * 2.0;
    auto t42 = t10 * t25 * t37 * 2.0;
    auto t43 = t41 + t42;

    Eigen::Matrix<double, 1, 3> retval;
    retval << t24 * t25 * t27 - t24 * t26 * t33 - t13 * t27 * t32 * t34 * (1.0 / 2.0) +
                t17 * t32 * t33 * t34 * (1.0 / 2.0) - t21 * t32 * t34 * t35 * (1.0 / 2.0),
      t24 * t26 * t35 - t24 * t27 * t36 + t13 * t27 * t34 * t38 * (1.0 / 2.0) - t17 * t33 * t34 * t38 * (1.0 / 2.0) +
        t21 * t34 * t35 * t38 * (1.0 / 2.0),
      -t24 * t25 * t35 + t24 * t33 * t36 + t13 * t27 * t34 * t43 * (1.0 / 2.0) - t17 * t33 * t34 * t43 * (1.0 / 2.0) +
        t21 * t34 * t35 * t43 * (1.0 / 2.0);
    return retval;
}

Eigen::Matrix<double, 1, 3> calculatePointToPlaneJpC(const double *const pt,
                                                     const double *const pA,
                                                     const double *const pB,
                                                     const double *const pC) {
    auto t17 = pA[0] * pB[1];
    auto t18 = pA[1] * pB[0];
    auto t19 = pA[0] * pC[1];
    auto t20 = pA[1] * pC[0];
    auto t21 = pB[0] * pC[1];
    auto t22 = pB[1] * pC[0];
    auto t2 = t17 - t18 - t19 + t20 + t21 - t22;
    auto t24 = pA[0] * pB[2];
    auto t25 = pA[2] * pB[0];
    auto t26 = pA[0] * pC[2];
    auto t27 = pA[2] * pC[0];
    auto t28 = pB[0] * pC[2];
    auto t29 = pB[2] * pC[0];
    auto t3 = t24 - t25 - t26 + t27 + t28 - t29;
    auto t5 = pA[1] * pB[2];
    auto t6 = pA[2] * pB[1];
    auto t7 = pA[1] * pC[2];
    auto t8 = pA[2] * pC[1];
    auto t9 = pB[1] * pC[2];
    auto t10 = pB[2] * pC[1];
    auto t4 = t5 - t6 - t7 + t8 + t9 - t10;
    auto t11 = pA[0] * pA[0];
    auto t12 = pB[0] * pB[0];
    auto t13 = pA[1] * pA[1];
    auto t14 = pB[2] * pB[2];
    auto t15 = pA[2] * pA[2];
    auto t16 = pB[1] * pB[1];
    auto t23 = t2 * t2;
    auto t30 = t3 * t3;
    auto t31 = t4 * t4;
    auto t32 = t23 + t30 + t31;
    auto t33 = 1.0 / std::pow(t32, 1.5);
    auto t34 = t11 * t16;
    auto t35 = t12 * t13;
    auto t36 = t11 * t14;
    auto t37 = t12 * t15;
    auto t38 = t13 * t14;
    auto t39 = t15 * t16;
    auto t40 = pt[0] * pC[0] * t13;
    auto t41 = pt[0] * pC[0] * t15;
    auto t42 = pt[1] * pC[1] * t11;
    auto t43 = pt[1] * pC[1] * t15;
    auto t44 = pt[2] * pC[2] * t11;
    auto t45 = pt[2] * pC[2] * t13;
    auto t46 = pt[0] * pC[0] * t16;
    auto t47 = pt[0] * pC[0] * t14;
    auto t48 = pt[1] * pC[1] * t12;
    auto t49 = pt[1] * pC[1] * t14;
    auto t50 = pt[2] * pC[2] * t12;
    auto t51 = pt[2] * pC[2] * t16;
    auto t52 = pt[0] * pA[0] * pA[1] * pB[1];
    auto t53 = pt[1] * pA[0] * pA[1] * pB[0];
    auto t54 = pt[0] * pA[0] * pA[2] * pB[2];
    auto t55 = pt[2] * pA[0] * pA[2] * pB[0];
    auto t56 = pt[1] * pA[1] * pA[2] * pB[2];
    auto t57 = pt[2] * pA[1] * pA[2] * pB[1];
    auto t58 = pt[0] * pA[1] * pB[0] * pB[1];
    auto t59 = pt[1] * pA[0] * pB[0] * pB[1];
    auto t60 = pt[0] * pA[2] * pB[0] * pB[2];
    auto t61 = pt[2] * pA[0] * pB[0] * pB[2];
    auto t62 = pt[1] * pA[2] * pB[1] * pB[2];
    auto t63 = pt[2] * pA[1] * pB[1] * pB[2];
    auto t64 = pt[0] * pA[0] * pB[1] * pC[1];
    auto t65 = pt[0] * pA[1] * pB[0] * pC[1];
    auto t66 = pt[1] * pA[0] * pB[1] * pC[0];
    auto t67 = pt[1] * pA[1] * pB[0] * pC[0];
    auto t68 = pt[0] * pA[0] * pB[2] * pC[2];
    auto t69 = pt[0] * pA[2] * pB[0] * pC[2];
    auto t70 = pt[2] * pA[0] * pB[2] * pC[0];
    auto t71 = pt[2] * pA[2] * pB[0] * pC[0];
    auto t72 = pt[1] * pA[1] * pB[2] * pC[2];
    auto t73 = pt[1] * pA[2] * pB[1] * pC[2];
    auto t74 = pt[2] * pA[1] * pB[2] * pC[1];
    auto t75 = pt[2] * pA[2] * pB[1] * pC[1];
    auto t76 = pA[0] * pA[1] * pB[0] * pC[1];
    auto t77 = pA[0] * pA[1] * pB[1] * pC[0];
    auto t78 = pA[0] * pA[2] * pB[0] * pC[2];
    auto t79 = pA[0] * pA[2] * pB[2] * pC[0];
    auto t80 = pA[1] * pA[2] * pB[1] * pC[2];
    auto t81 = pA[1] * pA[2] * pB[2] * pC[1];
    auto t82 = pA[0] * pB[0] * pB[1] * pC[1];
    auto t83 = pA[1] * pB[0] * pB[1] * pC[0];
    auto t84 = pA[0] * pB[0] * pB[2] * pC[2];
    auto t85 = pA[2] * pB[0] * pB[2] * pC[0];
    auto t86 = pA[1] * pB[1] * pB[2] * pC[2];
    auto t87 = pA[2] * pB[1] * pB[2] * pC[1];
    auto t89 = pt[0] * pA[0] * t16;
    auto t90 = pt[0] * pB[0] * t13;
    auto t91 = pt[0] * pA[0] * t14;
    auto t92 = pt[0] * pB[0] * t15;
    auto t93 = pt[1] * pA[1] * t12;
    auto t94 = pt[1] * pB[1] * t11;
    auto t95 = pt[1] * pA[1] * t14;
    auto t96 = pt[1] * pB[1] * t15;
    auto t97 = pt[2] * pA[2] * t12;
    auto t98 = pt[2] * pB[2] * t11;
    auto t99 = pt[2] * pA[2] * t16;
    auto t100 = pt[2] * pB[2] * t13;
    auto t101 = pA[0] * pC[0] * t16;
    auto t102 = pB[0] * pC[0] * t13;
    auto t103 = pA[0] * pC[0] * t14;
    auto t104 = pA[1] * pC[1] * t12;
    auto t105 = pB[1] * pC[1] * t11;
    auto t106 = pB[0] * pC[0] * t15;
    auto t107 = pA[1] * pC[1] * t14;
    auto t108 = pA[2] * pC[2] * t12;
    auto t109 = pB[2] * pC[2] * t11;
    auto t110 = pB[1] * pC[1] * t15;
    auto t111 = pA[2] * pC[2] * t16;
    auto t112 = pB[2] * pC[2] * t13;
    auto t113 = pt[0] * pA[0] * pA[1] * pC[1];
    auto t114 = pt[1] * pA[0] * pA[1] * pC[0];
    auto t115 = pt[0] * pA[0] * pA[2] * pC[2];
    auto t116 = pt[2] * pA[0] * pA[2] * pC[0];
    auto t117 = pt[1] * pA[1] * pA[2] * pC[2];
    auto t118 = pt[2] * pA[1] * pA[2] * pC[1];
    auto t119 = pt[0] * pA[1] * pB[1] * pC[0] * 2.0;
    auto t120 = pt[1] * pA[0] * pB[0] * pC[1] * 2.0;
    auto t121 = pA[0] * pA[1] * pB[0] * pB[1] * 2.0;
    auto t122 = pt[0] * pA[2] * pB[2] * pC[0] * 2.0;
    auto t123 = pt[2] * pA[0] * pB[0] * pC[2] * 2.0;
    auto t124 = pA[0] * pA[2] * pB[0] * pB[2] * 2.0;
    auto t125 = pt[1] * pA[2] * pB[2] * pC[1] * 2.0;
    auto t126 = pt[2] * pA[1] * pB[1] * pC[2] * 2.0;
    auto t127 = pA[1] * pA[2] * pB[1] * pB[2] * 2.0;
    auto t128 = pt[0] * pB[0] * pB[1] * pC[1];
    auto t129 = pt[1] * pB[0] * pB[1] * pC[0];
    auto t130 = pt[0] * pB[0] * pB[2] * pC[2];
    auto t131 = pt[2] * pB[0] * pB[2] * pC[0];
    auto t132 = pt[1] * pB[1] * pB[2] * pC[2];
    auto t133 = pt[2] * pB[1] * pB[2] * pC[1];
    auto t88 = t34 + t35 + t36 + t37 + t38 + t39 + t40 + t41 + t42 + t43 + t44 + t45 + t46 + t47 + t48 + t49 + t50 +
               t51 + t52 + t53 + t54 + t55 + t56 + t57 + t58 + t59 + t60 + t61 + t62 + t63 + t64 + t65 + t66 + t67 +
               t68 + t69 + t70 + t71 + t72 + t73 + t74 + t75 + t76 + t77 + t78 + t79 + t80 + t81 + t82 + t83 + t84 +
               t85 + t86 + t87 - t89 - t90 - t91 - t92 - t93 - t94 - t95 - t96 - t97 - t98 - t99 - t100 - t101 - t102 -
               t103 - t104 - t105 - t106 - t107 - t108 - t109 - t110 - t111 - t112 - t113 - t114 - t115 - t116 - t117 -
               t118 - t119 - t120 - t121 - t122 - t123 - t124 - t125 - t126 - t127 - t128 - t129 - t130 - t131 - t132 -
               t133;

    Eigen::Matrix<double, 1, 3> retval;
    retval << t4 * t33 * t88, -t3 * t33 * t88, t2 * t33 * t88;
    return retval;
}
}

namespace wave {

Eigen::Matrix3d calculatePointToLineWeight(const double *const pt,
                                           const double *const ptA,
                                           const double *const ptB,
                                           const double &variance) {
    Eigen::Matrix3d iso_covar = variance * Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Jr_pt = calculatePointToLineJpt(ptA, ptB);
    Eigen::Matrix3d Jr_pA = calculatePointToLineJpA(pt, ptA, ptB);
    Eigen::Matrix3d Jr_pB = calculatePointToLineJpB(pt, ptA, ptB);

    Eigen::Matrix3d prox_covar;
    prox_covar.noalias() = Jr_pt * iso_covar * Jr_pt.transpose() + Jr_pA * iso_covar * Jr_pA.transpose() +
                           Jr_pB * iso_covar * Jr_pB.transpose();

    return prox_covar.inverse().sqrt();
}

double calculatePointToPlaneWeight(const double *const pt,
                                   const double *const ptA,
                                   const double *const ptB,
                                   const double *const ptC,
                                   const double &variance) {
    Eigen::Matrix3d iso_covar = variance * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 1, 3> Jr_pt = calculatePointToPlaneJpt(ptA, ptB, ptC);
    Eigen::Matrix<double, 1, 3> Jr_pA = calculatePointToPlaneJpA(pt, ptA, ptB, ptC);
    Eigen::Matrix<double, 1, 3> Jr_pB = calculatePointToPlaneJpB(pt, ptA, ptB, ptC);
    Eigen::Matrix<double, 1, 3> Jr_pC = calculatePointToPlaneJpC(pt, ptA, ptB, ptC);

    Eigen::Matrix<double, 1, 1> prox_covar = Jr_pt * iso_covar * Jr_pt.transpose() + Jr_pA * iso_covar * Jr_pA.transpose() +
                 Jr_pB * iso_covar * Jr_pB.transpose() + Jr_pC * iso_covar * Jr_pC.transpose();

    return std::sqrt(1.0/prox_covar(0));
}
}