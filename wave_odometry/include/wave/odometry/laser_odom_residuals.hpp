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

        T den = ceres::sqrt(cBA_BC[0] * cBA_BC[0] + cBA_BC[1] * cBA_BC[1] +
                            cBA_BC[2] * cBA_BC[2]);
        T num = cBA_BC[0] * d_B[0] + cBA_BC[1] * d_B[1] + cBA_BC[2] * d_B[2];

        residuals[0] = num / den;

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

class AnalyticalPointToLine : public ceres::SizedCostFunction<1, 6> {
 public:
    virtual ~AnalyticalPointToLine() {}
    AnalyticalPointToLine(const double *const p,
                          const double *const pA,
                          const double *const pB,
                          const double *const scal) {
        this->pt = p;
        this->ptA = pA;
        this->ptB = pB;
        this->scale = scal;
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const {
        double r[3];
        r[0] = *(this->scale) * parameters[0][0];
        r[1] = *(this->scale) * parameters[0][1];
        r[2] = *(this->scale) * parameters[0][2];

        double point[3];
        ceres::AngleAxisRotatePoint(r, this->pt, point);
        // trans[3,4,5] are the translation.
        point[0] += this->scale[0] * parameters[0][3];
        point[1] += this->scale[0] * parameters[0][4];
        point[2] += this->scale[0] * parameters[0][5];

        double p_A[3] = {
          point[0] - ptA[0], point[1] - ptA[1], point[2] - ptA[2]};
        double p_B[3] = {
          point[0] - ptB[0], point[1] - ptB[1], point[2] - ptB[2]};
        double cross[3];
        ceres::CrossProduct(p_A, p_B, cross);

        double diff[3] = {ptA[0] - ptB[0], ptA[1] - ptB[1], ptA[2] - ptB[2]};
        double bottom = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];
        if (bottom < 1e-10) {
            // The points defining the line are too close to each other
            return false;
        }

        residuals[0] =
          (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]) / bottom;

        if (jacobians != NULL && jacobians[0] != NULL) {
            // compute jacobian
            // First is the l2norm squared
            Vec3 Jl2_C(2/bottom * Vec3(cross[0], cross[1], cross[2]));

            // Next is the cross product wrt to the transformed point
            Mat3 JC_P;
            JC_P(0, 0) = 0;
            JC_P(1, 0) = -diff[2];
            JC_P(2, 0) = diff[1];
            JC_P(0, 1) = diff[2];
            JC_P(1, 1) = 0;
            JC_P(2, 1) = -diff[0];
            JC_P(0, 2) = -diff[1];
            JC_P(1, 2) = diff[0];
            JC_P(2, 2) = 0;

            // Next there is the rotation wrt the angle-axis parameterization
            Vec3 wvec(r[0], r[1], r[2]);  // scaled rotation
            Vec3 P_noT(this->pt[0], this->pt[1], this->pt[2]);
            Mat3 J_R_w;
            Mat3 rot_mat = Rotation::expMapAndJacobian(wvec, J_R_w);
            Vec3 rotated_pt(rot_mat * P_noT);

            // Next is the transformed point wrt the rotation.
            Mat3 J_P_R;
            J_P_R(0, 0) = 0;
            J_P_R(1, 0) = -rotated_pt(2);
            J_P_R(2, 0) = rotated_pt(1);
            J_P_R(0, 1) = rotated_pt(2);
            J_P_R(1, 1) = 0;
            J_P_R(2, 1) = -rotated_pt(0);
            J_P_R(0, 2) = -rotated_pt(1);
            J_P_R(1, 2) = rotated_pt(0);
            J_P_R(2, 2) = 0;

            // Concatenate jacobians in correct order to get
            // the jacobians of the cross product wrt the angle axis parameters
            Mat3 JC_w = JC_P * J_P_R * J_R_w * scale[0];

            jacobians[0][0] =
              Jl2_C[0] * JC_w(0, 0) + Jl2_C[1] * JC_w(1, 0) + Jl2_C[2] * JC_w(2, 0);
            jacobians[0][1] =
              Jl2_C[0] * JC_w(0, 1) + Jl2_C[1] * JC_w(1, 1) + Jl2_C[2] * JC_w(2, 1);
            jacobians[0][2] =
              Jl2_C[0] * JC_w(0, 2) + Jl2_C[1] * JC_w(1, 2) + Jl2_C[2] * JC_w(2, 2);

            // The translation part of the transform is linear:
            jacobians[0][3] = scale[0] * Jl2_C[0];
            jacobians[0][4] = scale[0] * Jl2_C[1];
            jacobians[0][5] = scale[0] * Jl2_C[2];

            jacobians[0][3] =
                    Jl2_C[0] * JC_P(0, 0) + Jl2_C[1] * JC_P(1, 0) + Jl2_C[2] * JC_P(2, 0);
            jacobians[0][4] =
                    Jl2_C[0] * JC_P(0, 1) + Jl2_C[1] * JC_P(1, 1) + Jl2_C[2] * JC_P(2, 1);
            jacobians[0][5] =
                    Jl2_C[0] * JC_P(0, 2) + Jl2_C[1] * JC_P(1, 2) + Jl2_C[2] * JC_P(2, 2);

            jacobians[0][3] *= scale[0];
            jacobians[0][4] *= scale[0];
            jacobians[0][5] *= scale[0];
        }

        return true;
    }

 private:
    const double *pt;
    const double *ptA;
    const double *ptB;
    const double *scale;
};

class AnalyticalPointToPlane : public ceres::SizedCostFunction<1, 6> {
 public:
    virtual ~AnalyticalPointToPlane() {}
    AnalyticalPointToPlane(const double *const p,
                           const double *const pA,
                           const double *const pB,
                           const double *const pC,
                           const double *const scal) {
        this->pt = p;
        this->ptA = pA;
        this->ptB = pB;
        this->ptC = pC;
        this->scale = scal;
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const {
        double r[3];
        r[0] = *(this->scale) * parameters[0][0];
        r[1] = *(this->scale) * parameters[0][1];
        r[2] = *(this->scale) * parameters[0][2];

        double point[3];
        ceres::AngleAxisRotatePoint(r, this->pt, point);
        // trans[3,4,5] are the translation.
        point[0] += this->scale[0] * parameters[0][3];
        point[1] += this->scale[0] * parameters[0][4];
        point[2] += this->scale[0] * parameters[0][5];

        double d_B[3] = {
          point[0] - ptB[0], point[1] - ptB[1], point[2] - ptB[2]};
        double dBA[3] = {ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2]};
        double dBC[3] = {ptB[0] - ptC[0], ptB[1] - ptC[1], ptB[2] - ptC[2]};

        double cBA_BC[3];
        ceres::CrossProduct(dBA, dBC, cBA_BC);

        double den = ceres::sqrt(cBA_BC[0] * cBA_BC[0] + cBA_BC[1] * cBA_BC[1] +
                                 cBA_BC[2] * cBA_BC[2]);
        double num =
          cBA_BC[0] * d_B[0] + cBA_BC[1] * d_B[1] + cBA_BC[2] * d_B[2];

        residuals[0] = num / den;

        if (jacobians != NULL && jacobians[0] != NULL) {
            // compute jacobian
            // setup alias to make importing matlab string easier
            const double &XA1 = this->ptA[0], &XA2 = this->ptA[1],
                         &XA3 = this->ptA[2], &XB1 = this->ptB[0],
                         &XB2 = this->ptB[1], &XB3 = this->ptB[2],
                         &XC1 = this->ptC[0], &XC2 = this->ptC[1],
                         &XC3 = this->ptC[2];

            // clang-format off
            Vec3 D_P(
                    -((XA2-XB2)*(XB3-XC3)-(XA3-XB3)*(XB2-XC2))*1.0/sqrt(pow(((XA1-XB1)*(XB2-XC2)-(XA2-XB2)*(XB1-XC1)),2)+pow(((XA1-XB1)*(XB3-XC3)-(XA3-XB3)*(XB1-XC1)),2)+pow(((XA2-XB2)*(XB3-XC3)-(XA3-XB3)*(XB2-XC2)),2)),
                    ((XA1-XB1)*(XB3-XC3)-(XA3-XB3)*(XB1-XC1))*1.0/sqrt(pow(((XA1-XB1)*(XB2-XC2)-(XA2-XB2)*(XB1-XC1)),2)+pow(((XA1-XB1)*(XB3-XC3)-(XA3-XB3)*(XB1-XC1)),2)+pow(((XA2-XB2)*(XB3-XC3)-(XA3-XB3)*(XB2-XC2)),2)),
                    -((XA1-XB1)*(XB2-XC2)-(XA2-XB2)*(XB1-XC1))*1.0/sqrt(pow(((XA1-XB1)*(XB2-XC2)-(XA2-XB2)*(XB1-XC1)),2)+pow(((XA1-XB1)*(XB3-XC3)-(XA3-XB3)*(XB1-XC1)),2)+pow(((XA2-XB2)*(XB3-XC3)-(XA3-XB3)*(XB2-XC2)),2)));

            // clang-format on

            // The rotational parts contain the derivative of the coordinate
            // mapping composed with the derivative of the
            // exponential map

            Vec3 wvec(r[0], r[1], r[2]);  // scaled rotation
            Vec3 P_noT(this->pt[0], this->pt[1], this->pt[2]);
            Mat3 J_R_w;
            Mat3 rot_mat = Rotation::expMapAndJacobian(wvec, J_R_w);
            Vec3 rotated_pt(rot_mat * P_noT);
            Mat3 neg_skw;
            neg_skw(0, 0) = 0;
            neg_skw(1, 0) = -rotated_pt(2);
            neg_skw(2, 0) = rotated_pt(1);
            neg_skw(0, 1) = rotated_pt(2);
            neg_skw(1, 1) = 0;
            neg_skw(2, 1) = -rotated_pt(0);
            neg_skw(0, 2) = -rotated_pt(1);
            neg_skw(1, 2) = rotated_pt(0);
            neg_skw(2, 2) = 0;
            Mat3 temp = neg_skw * J_R_w * scale[0];

            jacobians[0][0] =
              D_P[0] * temp(0, 0) + D_P[1] * temp(1, 0) + D_P[2] * temp(2, 0);
            jacobians[0][1] =
              D_P[0] * temp(0, 1) + D_P[1] * temp(1, 1) + D_P[2] * temp(2, 1);
            jacobians[0][2] =
              D_P[0] * temp(0, 2) + D_P[1] * temp(1, 2) + D_P[2] * temp(2, 2);

            // The translation part of the transform is linear:
            jacobians[0][3] = scale[0] * D_P[0];
            jacobians[0][4] = scale[0] * D_P[1];
            jacobians[0][5] = scale[0] * D_P[2];
        }

        return true;
    }

 private:
    const double *pt;
    const double *ptA;
    const double *ptB;
    const double *ptC;
    const double *scale;
};

}  // namespace wave

#endif  // WAVE_LASER_ODOM_RESIDUALS_HPP
