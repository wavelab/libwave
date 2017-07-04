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
    PointToLineError(double scl, double *p, double *pA, double *pB)
        : scale(scl), pt(p), ptA(pA), ptB(pB) {}
    template <typename T>
    bool operator()(const T *const trans, T *residuals) const {
        // trans[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(trans, this->pt, p);
        // trans[3,4,5] are the translation.
        p[0] += trans[3];
        p[1] += trans[4];
        p[2] += trans[5];

        T int1 = (this->ptA[0] - this->pt[0]) * (this->ptB[1] - this->pt[1]) -
                 (this->ptA[1] - this->pt[1]) * (this->ptB[0] - this->pt[0]);
        T int2 = (this->ptA[0] - this->pt[0]) * (this->ptB[2] - this->pt[2]) -
                 (this->ptA[2] - this->pt[2]) * (this->ptB[0] - this->pt[0]);
        T int3 = (this->ptA[1] - this->pt[1]) * (this->ptB[2] - this->pt[2]) -
                 (this->ptA[2] - this->pt[2]) * (this->ptB[1] - this->pt[1]);
        T diff[3] = {this->ptA[0] - this->ptB[0],
                     this->ptA[1] - this->ptB[1],
                     this->ptA[2] - this->ptB[2]};

        residuals[0] = std::sqrt(
          (int1 * int1 + int2 * int2 + int3 * int3) /
          (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]));

        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const double scale,
                                       const double *const point,
                                       const double *const endpt1,
                                       const double *const endpt2) {
        return (new ceres::AutoDiffCostFunction<PointToLineError, 1, 6>(
          new PointToLineError(scale, point, endpt1, endpt2)));
    }
    double scale;
    double pt[3];
    double ptA[3];
    double ptB[3];
};

struct PointToPlaneError {
    PointToPlaneError(double scl, double *p, double *pA, double *pB, double *pC)
        : scale(scl), pt(p), ptA(pA), ptB(pB), ptC(pC) {}
    template <typename T>
    bool operator()(const T *const trans, T *residuals) const {
        // trans[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(trans, this->pt, p);
        // trans[3,4,5] are the translation.
        p[0] += trans[3];
        p[1] += trans[4];
        p[2] += trans[5];

        T int1 = (this->ptA[0] - this->ptB[0]) * (this->ptB[1] - this->ptC[1]) -
                 (this->ptA[1] - this->ptB[1]) * (this->ptB[0] - this->ptC[0]);
        T int2 = (this->ptA[0] - this->ptB[0]) * (this->ptB[2] - this->ptC[2]) -
                 (this->ptA[2] - this->ptB[2]) * (this->ptB[0] - this->ptC[0]);
        T int3 = (this->ptA[1] - this->ptB[1]) * (this->ptB[2] - this->ptC[2]) -
                 (this->ptA[2] - this->ptB[2]) * (this->ptB[1] - this->ptC[1]);
        T int4 = (this->ptB[0] - this->pt[0]) * (this->ptB[0] - this->pt[0]) +
                 (this->ptB[1] - this->pt[1]) * (this->ptB[1] - this->pt[1]) +
                 (this->ptB[2] - this->pt[2]) * (this->ptB[2] - this->pt[2]);

        residuals[0] =
          std::sqrt((int4 + int1 * int1 + int2 * int2 + int3 * int3) /
                    (int1 * int1 + int2 * int2 + int3 * int3));

        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const double scale,
                                       const double *const point,
                                       const double *const plpt1,
                                       const double *const plpt2,
                                       const double *const plpt3) {
        return (new ceres::AutoDiffCostFunction<PointToLineError, 1, 6>(
          new PointToLineError(scale, point, plpt1, plpt2, plpt3)));
    }
    double scale;
    double pt[3];
    double ptA[3];
    double ptB[3];
    double ptC[3];
};
}

#endif  // WAVE_LASER_ODOM_RESIDUALS_HPP
