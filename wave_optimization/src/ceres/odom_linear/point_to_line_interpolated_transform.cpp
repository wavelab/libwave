#include "wave/optimization/ceres/odom_linear/point_to_line_interpolated_transform.hpp"
#include <Eigen/QR>

namespace wave {

SE3PointToLine::SE3PointToLine(const double *const p, const double *const pA, const double *const pB, const double *const scal, const Mat3 &CovZ, bool calculate_weight)
        : pt(p), ptA(pA), ptB(pB), scale(scal) {
    Eigen::Matrix<double, 3, 12> JP_T;
    Eigen::Matrix<double, 3, 3> Jres_P;

    JP_T << this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1, 0, 0, //
            0, this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1, 0, //
            0, 0, this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1;

    this->diff[0] = this->ptB[0] - this->ptA[0];
    this->diff[1] = this->ptB[1] - this->ptA[1];
    this->diff[2] = this->ptB[2] - this->ptA[2];
    this->bottom = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];

    if (this->bottom < 1e-10) {
        // The points defining the line are too close to each other
        throw std::out_of_range("Points defining line are too close!");
    }

    Jres_P(0,0) = 1 - (diff[0] * diff[0] / bottom);
    Jres_P(0,1) = -(diff[0] * diff[1] / bottom);
    Jres_P(0,2) = -(diff[0] * diff[2] / bottom);
    Jres_P(1,0) = -(diff[1] * diff[0] / bottom);
    Jres_P(1,1) = 1 - (diff[1] * diff[1] / bottom);
    Jres_P(1,2) = -(diff[1] * diff[2] / bottom);
    Jres_P(2,0) = -(diff[2] * diff[0] / bottom);
    Jres_P(2,1) = -(diff[2] * diff[1] / bottom);
    Jres_P(2,2) = 1 -(diff[2] * diff[2] / bottom);

    Eigen::Vector3d unitdiff;
    double invlength = 1.0/sqrt(this->bottom);
    if(this->diff[2] > 0) {
        unitdiff[0] = this->diff[0] * invlength;
        unitdiff[1] = this->diff[1] * invlength;
        unitdiff[2] = this->diff[2] * invlength;
    } else {
        unitdiff[0] = -this->diff[0] * invlength;
        unitdiff[1] = -this->diff[1] * invlength;
        unitdiff[2] = -this->diff[2] * invlength;
    }

    Eigen::Vector3d unitz;
    unitz << 0, 0, 1;

    auto v = unitdiff.cross(unitz);
    auto s = v.norm();
    auto c = unitz.dot(unitdiff);
    Mat3 skew;
    Transformation<>::skewSymmetric3(v, skew);
    this->rotation = Eigen::Matrix3d::Identity() + skew + skew*skew*((1-c)/(s*s));

    this->Jres_T = this->rotation * Jres_P * JP_T;

    if(calculate_weight) {
        auto rotated = (this->rotation * Jres_P * CovZ * Jres_P.transpose() * this->rotation.transpose());
        this->weight_matrix = rotated.block<2,2>(0,0).inverse().sqrt();
    } else {
        this->weight_matrix.setIdentity();
    }
}

bool SE3PointToLine::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Mat34> Tmap(parameters[0], 3, 4);
    Transformation<Eigen::Map<const Mat34>> Tk(Tmap);

    Transformation<Eigen::Matrix<double, 3, 4>> interpolated;

    Eigen::Map<const Vec3> PT(pt, 3, 1);
    auto twist = Tk.logMap();
    interpolated.setFromExpMap(*(this->scale) * twist);
    Vec3 POINT = interpolated.transform(PT);
    double point[3];
    Eigen::Map<Vec3>(point, 3, 1) = POINT;

    double p_A[3] = {point[0] - this->ptA[0], point[1] - this->ptA[1], point[2] - this->ptA[2]};

    double scaling = ceres::DotProduct(p_A, diff);
    // point on line closest to point
    double p_Tl[3] = {this->ptA[0] + (scaling / bottom) * diff[0],
                      this->ptA[1] + (scaling / bottom) * diff[1],
                      this->ptA[2] + (scaling / bottom) * diff[2]};

    double residual_o[3];
    residual_o[0] = point[0] - p_Tl[0];
    residual_o[1] = point[1] - p_Tl[1];
    residual_o[2] = point[2] - p_Tl[2];
    Eigen::Map<Vec3> res(residual_o, 3, 1);
    Eigen::Vector2d reduced = (this->rotation * res).block<2,1>(0,0);
    reduced = this->weight_matrix * reduced;
    Eigen::Map<Eigen::Vector2d>(residuals, 2, 1) = reduced;

    if (jacobians != NULL) {
        // Have to apply the "lift" jacobian to the Interpolation Jacobian because
        // of Ceres local parameterization
        Transformation<>::Jinterpolated(twist, *(this->scale), this->J_int);
        interpolated.J_lift(this->J_lift);
        Tk.J_lift(this->J_lift_full);

        this->J_lift_full_pinv = (this->J_lift_full.transpose() * this->J_lift_full).inverse() * this->J_lift_full.transpose();

        this->Jr_T = this->Jres_T * this->J_lift * this->J_int * this->J_lift_full_pinv;
        this->Jr_T_reduced = this->weight_matrix * this->Jr_T.block<2,12>(0,0);

        Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>>(jacobians[0], 2, 12) = this->Jr_T_reduced;
    }

    return true;
}

}  // namespace wave
