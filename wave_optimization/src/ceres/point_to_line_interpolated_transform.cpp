#include "wave/optimization/ceres/point_to_line_interpolated_transform.hpp"
#include <Eigen/QR>

namespace wave {

SE3PointToLine::SE3PointToLine(const double *const p, const double *const pA, const double *const pB, const double *const scal, const Mat3 &CovZ, bool calculate_weight)
        : pt(p), ptA(pA), ptB(pB), scale(scal) {
    this->JP_T << this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1, 0, 0, //
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

    this->Jres_P(0,0) = 1 - (diff[0] * diff[0] / bottom);
    this->Jres_P(0,1) = -(diff[0] * diff[1] / bottom);
    this->Jres_P(0,2) = -(diff[0] * diff[2] / bottom);
    this->Jres_P(1,0) = -(diff[1] * diff[0] / bottom);
    this->Jres_P(1,1) = 1 - (diff[1] * diff[1] / bottom);
    this->Jres_P(1,2) = -(diff[1] * diff[2] / bottom);
    this->Jres_P(2,0) = -(diff[2] * diff[0] / bottom);
    this->Jres_P(2,1) = -(diff[2] * diff[1] / bottom);
    this->Jres_P(2,2) = 1 -(diff[2] * diff[2] / bottom);

    if(calculate_weight) {
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
        auto skew = Transformation::skewSymmetric3(v);
        this->rotation = Eigen::Matrix3d::Identity() + skew + skew*skew*((1-c)/(s*s));

        auto rotated = (this->rotation * this->Jres_P * CovZ * this->Jres_P.transpose() * this->rotation.transpose());
        this->weight_matrix = rotated.block<2,2>(0,0).inverse().sqrt();
    } else {
        this->weight_matrix.setIdentity();
    }
}

bool SE3PointToLine::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Mat4 transform_matrix = Mat4::Identity();
    transform_matrix << parameters[0][0], parameters[0][3], parameters[0][6], parameters[0][9], parameters[0][1],
      parameters[0][4], parameters[0][7], parameters[0][10], parameters[0][2], parameters[0][5], parameters[0][8],
      parameters[0][11], 0, 0, 0, 1;
    Transformation transform;
    transform.setFromMatrix(transform_matrix);
    Transformation interpolated;

    Eigen::Map<const Vec3> PT(pt, 3, 1);
    auto twist = transform.logMap();
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
    Eigen::Map<Vec3> res(residuals, 3, 1);
    Eigen::Vector2d reduced = (this->rotation * res).block<2,1>(0,0);
    reduced = this->weight_matrix * reduced;
    Eigen::Map<Vec3>(residuals, 2, 1) = reduced;

    if (jacobians != NULL) {
        // Have to apply the "lift" jacobian to the Interpolation Jacobian because
        // of Ceres local parameterization
        Transformation::Jinterpolated(twist, *(this->scale), this->J_int);
        interpolated.J_lift(this->J_lift);
        transform.J_lift(this->J_lift_full);

        this->J_lift_full_pinv = (this->J_lift_full.transpose() * this->J_lift_full).inverse() * this->J_lift_full.transpose();

        this->Jr_T =
          this->weight_matrix * this->rotation * this->Jres_P * this->JP_T * this->J_lift * this->J_int * this->J_lift_full_pinv;

        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>>(jacobians[0], 3, 12) = this->Jr_T;
    }

    return true;
}

}  // namespace wave
