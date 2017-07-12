/** @file
 * @ingroup optimization
 *
 * This file contains the residual definitions for laser odometry as defined in
 * LOAM
 */

#ifndef WAVE_LASER_ODOM_RESIDUALS_HPP
#define WAVE_LASER_ODOM_RESIDUALS_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include "wave/geometry/rotation.hpp"

namespace wave {

struct PointToLineError {
    PointToLineError() {}
    template <typename T>
    bool operator()(const T *const trans,
                    const T *const pt,
                    const T *const ptA,
                    const T *const ptB,
                    const T *const scale,
                    T *residuals) const {
        // trans[0,1,2] are the angle-axis rotation.
        // trans[3,4,5] are the translation.
        // pt[0,1,2] is the point
        // ptA[0,1,2] & ptB[0,1,2] define the line
        // scale scales the transform
        T p[3];
        T w[3] = {
          scale[0] * trans[0], scale[0] * trans[1], scale[0] * trans[2]};
        ceres::AngleAxisRotatePoint(w, pt, p);
        // trans[3,4,5] are the translation.
        p[0] += scale[0] * trans[3];
        p[1] += scale[0] * trans[4];
        p[2] += scale[0] * trans[5];

        T int1 =
          (ptA[0] - p[0]) * (ptB[1] - p[1]) - (ptA[1] - p[1]) * (ptB[0] - p[0]);
        T int2 =
          (ptA[0] - p[0]) * (ptB[2] - p[2]) - (ptA[2] - p[2]) * (ptB[0] - p[0]);
        T int3 =
          (ptA[1] - p[1]) * (ptB[2] - p[2]) - (ptA[2] - p[2]) * (ptB[1] - p[1]);
        T diff[3] = {ptA[0] - ptB[0], ptA[1] - ptB[1], ptA[2] - ptB[2]};

        residuals[0] = ceres::sqrt(
          (int1 * int1 + int2 * int2 + int3 * int3) /
          (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]));

        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create() {
        return (
          new ceres::AutoDiffCostFunction<PointToLineError, 1, 6, 3, 3, 3, 1>(
            new PointToLineError()));
    }
};

struct PointToPlaneError {
    PointToPlaneError() {}
    template <typename T>
    bool operator()(const T *const trans,
                    const T *const pt,
                    const T *const ptA,
                    const T *const ptB,
                    const T *const ptC,
                    const T *const scale,
                    T *residuals) const {
        // trans[0,1,2] are the angle-axis rotation.
        T p[3];
        T w[3] = {
          scale[0] * trans[0], scale[0] * trans[1], scale[0] * trans[2]};
        ceres::AngleAxisRotatePoint(w, pt, p);
        // trans[3,4,5] are the translation.
        p[0] += scale[0] * trans[3];
        p[1] += scale[0] * trans[4];
        p[2] += scale[0] * trans[5];

        T d_B[3] = {p[0] - ptB[0], p[1] - ptB[1], p[2] - ptB[2]};
        T dBA[3] = {ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2]};
        T dBC[3] = {ptB[0] - ptC[0], ptB[1] - ptC[1], ptB[2] - ptC[2]};

        T cBA_BC[3];
        ceres::CrossProduct(dBA, dBC, cBA_BC);

        T den = ceres::sqrt(cBA_BC[0]*cBA_BC[0] + cBA_BC[1]*cBA_BC[1] + cBA_BC[2]*cBA_BC[2]);
        T num = cBA_BC[0]*d_B[0] + cBA_BC[1]*d_B[1] + cBA_BC[2]*d_B[2];

        residuals[0] = num/den;

//        T int1 = (ptA[0] - ptB[0]) * (ptB[1] - ptC[1]) -
//                 (ptA[1] - ptB[1]) * (ptB[0] - ptC[0]);
//        T int2 = (ptA[0] - ptB[0]) * (ptB[2] - ptC[2]) -
//                 (ptA[2] - ptB[2]) * (ptB[0] - ptC[0]);
//        T int3 = (ptA[1] - ptB[1]) * (ptB[2] - ptC[2]) -
//                 (ptA[2] - ptB[2]) * (ptB[1] - ptC[1]);
//        T int4 = (ptB[0] - p[0]) * (ptB[0] - p[0]) +
//                 (ptB[1] - p[1]) * (ptB[1] - p[1]) +
//                 (ptB[2] - p[2]) * (ptB[2] - p[2]);
//
//        residuals[0] =
//          ceres::sqrt((int4 + int1 * int1 + int2 * int2 + int3 * int3) /
//                      (int1 * int1 + int2 * int2 + int3 * int3));

        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create() {
        return (new ceres::
                  AutoDiffCostFunction<PointToPlaneError, 1, 6, 3, 3, 3, 3, 1>(
                    new PointToPlaneError()));
    }
};
}

#endif  // WAVE_LASER_ODOM_RESIDUALS_HPP
