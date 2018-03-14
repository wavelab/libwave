#include "wave/optimization/ceres/odom_linear/point_to_plane_interpolated_transform.hpp"

namespace wave {

SE3PointToPlane::SE3PointToPlane(const double *const p,
                const double *const pA,
                const double *const pB,
                const double *const pC,
                const double *const scal,
                const Mat3 &covZ,
                bool use_weighting)
        : pt(p), ptA(pA), ptB(pB), ptC(pC), scale(scal) {
    Eigen::Matrix<double, 3, 12> JP_Ti;
    Eigen::Matrix<double, 1, 3> Jr_P;

    JP_Ti << this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1, 0, 0, //
            0, this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1, 0, //
            0, 0, this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1;

    this->calculateJr_P(Jr_P);
    this->Jr_Ti = Jr_P * JP_Ti;

    if (use_weighting) {
        double covR = (Jr_P * covZ * Jr_P.transpose())(0);
        this->weight = std::sqrt(1.0/covR);
    } else {
        this->weight = 1;
    }
}

bool SE3PointToPlane::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Mat34> Tmap(parameters[0], 3, 4);
    Transformation<Eigen::Map<const Mat34>> Tk(Tmap);

    Transformation<Eigen::Matrix<double, 3, 4>> interpolated;

    Eigen::Map<const Vec3> PT(pt, 3, 1);
    auto twist = Tk.logMap();
    interpolated.setFromExpMap(*(this->scale) * twist);
    Vec3 POINT = interpolated.transform(PT);
    double point[3];
    Eigen::Map<Vec3>(point, 3, 1) = POINT;

    //point is the transformed point.
    double d_B[3] = {point[0] - ptB[0], point[1] - ptB[1], point[2] - ptB[2]};
    double dBA[3] = {ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2]};
    double dBC[3] = {ptB[0] - ptC[0], ptB[1] - ptC[1], ptB[2] - ptC[2]};

    double cBA_BC[3];
    ceres::CrossProduct(dBA, dBC, cBA_BC);

    double den = ceres::sqrt(cBA_BC[0] * cBA_BC[0] + cBA_BC[1] * cBA_BC[1] + cBA_BC[2] * cBA_BC[2]);
    double num = cBA_BC[0] * d_B[0] + cBA_BC[1] * d_B[1] + cBA_BC[2] * d_B[2];

    residuals[0] = this->weight * (num / den);

    if((jacobians != NULL) && (jacobians[0] != NULL)) {
        // Have to apply the "lift" jacobian to the Interpolation Jacobian because
        // of Ceres local parameterization
        Transformation<Eigen::Matrix<double, 3, 4>>::Jinterpolated(twist, *(this->scale), this->J_int);
        interpolated.J_lift(this->J_lift);
        Tk.J_lift(this->J_lift_full);

        this->J_lift_full_pinv = (this->J_lift_full.transpose() * this->J_lift_full).inverse() * this->J_lift_full.transpose();

        // Identity * scale is approximating the Jacobian of the Interpolated Transform wrt the complete Transform
        this->Jr_T = this->weight * this->Jr_Ti * J_lift * J_int * this->J_lift_full_pinv;

        Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>>(jacobians[0], 1, 12) = this->Jr_T;
    }

    return true;
}

/// Jacobian of the residual wrt the transformed point
void SE3PointToPlane::calculateJr_P(Eigen::Matrix<double, 1, 3> &Jr_P) const {
    const double &XA1 = this->ptA[0], &XA2 = this->ptA[1], &XA3 = this->ptA[2], &XB1 = this->ptB[0],
            &XB2 = this->ptB[1], &XB3 = this->ptB[2], &XC1 = this->ptC[0], &XC2 = this->ptC[1],
            &XC3 = this->ptC[2];

    Jr_P << -((XA2-XB2)*(XB3-XC3)-(XA3-XB3)*(XB2-XC2))*1.0/sqrt(pow(((XA1-XB1)*(XB2-XC2)-(XA2-XB2)*(XB1-XC1)),2)+pow(((XA1-XB1)*(XB3-XC3)-(XA3-XB3)*(XB1-XC1)),2)+pow(((XA2-XB2)*(XB3-XC3)-(XA3-XB3)*(XB2-XC2)),2)),
            ((XA1-XB1)*(XB3-XC3)-(XA3-XB3)*(XB1-XC1))*1.0/sqrt(pow(((XA1-XB1)*(XB2-XC2)-(XA2-XB2)*(XB1-XC1)),2)+pow(((XA1-XB1)*(XB3-XC3)-(XA3-XB3)*(XB1-XC1)),2)+pow(((XA2-XB2)*(XB3-XC3)-(XA3-XB3)*(XB2-XC2)),2)),
            -((XA1-XB1)*(XB2-XC2)-(XA2-XB2)*(XB1-XC1))*1.0/sqrt(pow(((XA1-XB1)*(XB2-XC2)-(XA2-XB2)*(XB1-XC1)),2)+pow(((XA1-XB1)*(XB3-XC3)-(XA3-XB3)*(XB1-XC1)),2)+pow(((XA2-XB2)*(XB3-XC3)-(XA3-XB3)*(XB2-XC2)),2));
}

}  // namespace wave
