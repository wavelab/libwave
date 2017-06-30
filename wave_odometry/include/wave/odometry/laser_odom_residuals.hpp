/** @file
 * @ingroup optimization
 *
 * This file contains the residual definitions for laser odometry as defined in
 * LOAM
 */

#ifndef WAVE_LASER_ODOM_RESIDUALS_HPP
#define WAVE_LASER_ODOM_RESIDUALS_HPP

#include <ceres/ceres.h>
#include <Eigen/Core>
#include "wave/geometry/rotation.hpp"

namespace wave {

class PointToLineFunction : public ceres::SizedCostFunction<1, 6> {
 public:
    float scale;
    float pt[3];
    float ptA[3];
    float ptB[3];

    PointToLineFunction(float scl, float p[], float pA[], float pB[])
        : scale(scl), pt(p), ptA(pA), ptB(pB){};
    virtual ~PointToLineFunction() {}
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const {
        Rotation R;
        Vec3 w(parameters[0][3], parameters[0][4], parameters[0][5]);
        R.setFromExpMap(this->scale * w);

        // aliases to make matlab copy-paste easier
        // clang-format off
        float T1 = this->scale * parameters[0][0],
                T2 = this->scale * parameters[0][1],
                T3 = this->scale * parameters[0][2];
        float &X1 = this->pt[0],
            &X2 = this->pt[1],
            &X3 = this->pt[2];
        float &XA1 = this->ptA[0],
            &XA2 = this->ptA[1],
            &XA3 = this->ptA[2];
        float &XB1 = this->ptB[0],
            &XB2 = this->ptB[1],
            &XB3 = this->ptB[2];
        auto Rmat = R.toRotationMatrix();

        // clang-format on
        float inter1 =
          ((T1 - XA1 + Rmat(0, 0) * X1 + Rmat(0, 1) * X2 + Rmat(0, 2) * X3) *
             (T2 - XB2 + Rmat(1, 0) * X1 + Rmat(1, 1) * X2 + Rmat(1, 2) * X3) -
           (T2 - XA2 + Rmat(1, 0) * X1 + Rmat(1, 1) * X2 + Rmat(1, 2) * X3) *
             (T1 - XB1 + Rmat(0, 0) * X1 + Rmat(0, 1) * X2 + Rmat(0, 2) * X3));

        float inter2 = ((T1 - XA1 + Rmat(0,0) * X1 + Rmat(0,1) * X2 + Rmat(0,2) * X3) *
                          (T3 - XB3 + Rmat(2,0) * X1 + Rmat(2,1) * X2 + Rmat(2,2) * X3) -
                        (T3 - XA3 + Rmat(2,0) * X1 + Rmat(2,1) * X2 + Rmat(2,2) * X3) *
                          (T1 - XB1 + Rmat(0,0) * X1 + Rmat(0,1) * X2 + Rmat(0,2) * X3));

        float inter3 = ((T2 - XA2 + Rmat(1,0) * X1 + Rmat(1,1) * X2 + Rmat(1,2) * X3) *
                          (T3 - XB3 + Rmat(2,0) * X1 + Rmat(2,1) * X2 + Rmat(2,2) * X3) -
                        (T3 - XA3 + Rmat(2,0) * X1 + Rmat(2,1) * X2 + Rmat(2,2) * X3) *
                          (T2 - XB2 + Rmat(1,0) * X1 + Rmat(1,1) * X2 + Rmat(1,2) * X3));

        float inter4 = ((XA1 - XB1)*(XA1 - XB1) + (XA2 - XB2)*(XA2 - XB2) + (XA3 - XB3)*(XA3 - XB3));

        if (inter4 == 0f) {
            return false;
        }

        residuals[0] = sqrt((inter1 * inter1 + inter2 * inter2 + inter3*inter3) / inter4);

        if (jacobians != NULL && jacobians[0] != NULL) {
            Mat3 J_exp = kindr::getJacobianOfExponentialMap(w);
        }
    }
};
}

#endif  // WAVE_LASER_ODOM_RESIDUALS_HPP
