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
#include <ceres/local_parameterization.h>
#include <Eigen/Core>
#include "wave/geometry/rotation.hpp"

namespace wave {

//const double unit_x[3] = {1, 0, 0};
//const double unit_y[3] = {0, 1, 0};
// Point to Line cost has a null direction of size 1. This parameterization could be used to solve small problem
// examples
//class PointToLineParameterization : public ceres::LocalParameterization {
// public:
//    explicit PointToLineParameterization(const double *const pA, const double *const pB) {
//        double line[3] = {pA[0] - pB[0], pA[1] - pB[1], pA[2] - pB[2]};
//        double length = ceres::sqrt(line[0]*line[0] + line[1]*line[1] + line[2]*line[2]);
//        line[0] /= length;
//        line[1] /= length;
//        line[2] /= length;
//        // now have unit vector in direction of line, this is the null direction of the cost.
//        // Now going to find rotation between unit z vector (0,0,1) and the direction of the line.
//        double unit_z[3] = {0, 0, 1};
//        double magnitude = std::acos(line[2]);
//        double axis[3];
//        ceres::CrossProduct(line, unit_z, axis);
//        //scale axis to have length magnitude
//        double axis_magnitude = ceres::sqrt(axis[0]*axis[0] + axis[1]*axis[1] +axis[2]*axis[2]);
//        double scale = magnitude/axis_magnitude;
//        axis[0] *= scale;
//        axis[1] *= scale;
//        axis[2] *= scale;
//        ceres::AngleAxisRotatePoint(axis, unit_x, this->v1);
//        ceres::AngleAxisRotatePoint(axis, unit_y, this->v2);
//    }
//    virtual ~PointToLineParameterization() {}
//    virtual bool Plus(const double* x,
//                    const double* delta,
//                    double* x_plus_delta) const {
//        x_plus_delta[0] = x[0] + delta[0]*this->v1[0] + delta[1]*this->v2[0];
//        x_plus_delta[1] = x[1] + delta[0]*this->v1[1] + delta[1]*this->v2[1];
//        x_plus_delta[2] = x[2] + delta[0]*this->v1[2] + delta[1]*this->v2[2];
//        return true;
//    }
//    virtual bool ComputeJacobian(const double* x, double* jacobian) const {
//        jacobian[0] = this->v1[0];
//        jacobian[0] = this->v1[1];
//        jacobian[0] = this->v1[2];
//        jacobian[0] = this->v2[0];
//        jacobian[0] = this->v2[1];
//        jacobian[0] = this->v2[2];
//        return true;
//    }
//    virtual int GlobalSize() const {return 3;}
//    virtual int LocalSize() const {return 2;}
// private:
//    double v1[3];
//    double v2[3];
//};

class AnalyticalPointToLine : public ceres::SizedCostFunction<3, 3, 3> {
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
        point[0] += this->scale[0] * parameters[1][0];
        point[1] += this->scale[0] * parameters[1][2];
        point[2] += this->scale[0] * parameters[1][3];

        double p_A[3] = {
          point[0] - ptA[0], point[1] - ptA[1], point[2] - ptA[2]};

        double diff[3] = {ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2]};
        double bottom =
          diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];
        if (bottom < 1e-10) {
            // The points defining the line are too close to each other
            return false;
        }

        double scaling = ceres::DotProduct(p_A, diff);
        // point on line closest to point
        double p_Tl[3] = {ptA[0] + (scaling / bottom) * diff[0],
                          ptA[1] + (scaling / bottom) * diff[1],
                          ptA[2] + (scaling / bottom) * diff[2]};

        residuals[0] = point[0] - p_Tl[0];
        residuals[1] = point[1] - p_Tl[1];
        residuals[2] = point[2] - p_Tl[2];

        if (jacobians != NULL) {
            // compute jacobian
            // First jacobian wrt the transformed point
            Mat3 Jres_P;
            Jres_P(0, 0) = 1 - (diff[0] * diff[0] / bottom);
            Jres_P(1, 0) = -(diff[1] * diff[0] / bottom);
            Jres_P(2, 0) = -(diff[2] * diff[0] / bottom);
            Jres_P(0, 1) = -(diff[0] * diff[1] / bottom);
            Jres_P(1, 1) = 1 - (diff[1] * diff[1] / bottom);
            Jres_P(2, 1) = -(diff[2] * diff[1] / bottom);
            Jres_P(0, 2) = -(diff[0] * diff[2] / bottom);
            Jres_P(1, 2) = -(diff[1] * diff[2] / bottom);
            Jres_P(2, 2) = 1 - (diff[2] * diff[2] / bottom);

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
            Mat3 Jres_w = Jres_P * J_P_R * J_R_w * scale[0];
            Jres_w.transpose();

            if (jacobians[0] != NULL) {
                jacobians[0][0] = Jres_w(0, 0);
                jacobians[0][1] = Jres_w(0, 1);
                jacobians[0][2] = Jres_w(0, 2);
                jacobians[0][3] = Jres_w(1, 0);
                jacobians[0][4] = Jres_w(1, 1);
                jacobians[0][5] = Jres_w(1, 2);
                jacobians[0][6] = Jres_w(2, 0);
                jacobians[0][7] = Jres_w(2, 1);
                jacobians[0][8] = Jres_w(2, 2);
            }

            if (jacobians[1] != NULL) {
                jacobians[1][0] = Jres_P(0, 0) * scale[0];
                jacobians[1][1] = Jres_P(0, 1) * scale[0];
                jacobians[1][2] = Jres_P(0, 2) * scale[0];
                jacobians[1][3] = Jres_P(1, 0) * scale[0];
                jacobians[1][4] = Jres_P(1, 1) * scale[0];
                jacobians[1][5] = Jres_P(1, 2) * scale[0];
                jacobians[1][6] = Jres_P(2, 0) * scale[0];
                jacobians[1][7] = Jres_P(2, 1) * scale[0];
                jacobians[1][8] = Jres_P(2, 2) * scale[0];
            }
        }

        return true;
    }

 private:
    const double *pt;
    const double *ptA;
    const double *ptB;
    const double *scale;
};

class AnalyticalPointToPlane : public ceres::SizedCostFunction<1, 3, 3> {
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
        point[0] += this->scale[0] * parameters[1][0];
        point[1] += this->scale[0] * parameters[1][2];
        point[2] += this->scale[0] * parameters[1][3];

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

        if (jacobians != NULL) {
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

            if (jacobians[0] != NULL) {
                jacobians[0][0] =
                        D_P[0] * temp(0, 0) + D_P[1] * temp(1, 0) + D_P[2] * temp(2, 0);
                jacobians[0][1] =
                        D_P[0] * temp(0, 1) + D_P[1] * temp(1, 1) + D_P[2] * temp(2, 1);
                jacobians[0][2] =
                        D_P[0] * temp(0, 2) + D_P[1] * temp(1, 2) + D_P[2] * temp(2, 2);
            }

            if (jacobians[1] != NULL) {
                // The translation part of the transform is linear:
                jacobians[1][0] = scale[0] * D_P[0];
                jacobians[1][1] = scale[0] * D_P[1];
                jacobians[1][2] = scale[0] * D_P[2];
            }
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
