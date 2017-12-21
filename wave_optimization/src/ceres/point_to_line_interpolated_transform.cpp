#include "wave/optimization/ceres/point_to_line_interpolated_transform.hpp"
#include <Eigen/QR>

namespace wave {

bool SE3PointToLine::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Mat4 transform_matrix = Mat4::Identity();
    transform_matrix << parameters[0][0], parameters[0][3], parameters[0][6], parameters[0][9],
            parameters[0][1], parameters[0][4], parameters[0][7], parameters[0][10],
            parameters[0][2], parameters[0][5], parameters[0][8], parameters[0][11],
            0, 0, 0, 1;
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

    double diff[3] = {this->ptB[0] - this->ptA[0], this->ptB[1] - this->ptA[1], this->ptB[2] - this->ptA[2]};
    double bottom = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];
    if (bottom < 1e-10) {
        // The points defining the line are too close to each other
        return false;
    }

    double scaling = ceres::DotProduct(p_A, diff);
    // point on line closest to point
    double p_Tl[3] = {this->ptA[0] + (scaling / bottom) * diff[0],
                      this->ptA[1] + (scaling / bottom) * diff[1],
                      this->ptA[2] + (scaling / bottom) * diff[2]};

    residuals[0] = point[0] - p_Tl[0];
    residuals[1] = point[1] - p_Tl[1];
    residuals[2] = point[2] - p_Tl[2];
    Eigen::Map<Vec3> res(residuals, 3, 1);
    res = this->weight_matrix * res;
    Eigen::Map<Vec3>(residuals, 3, 1) = res;

    if (jacobians != NULL) {
        // This is the Jacobian of the cost function wrt the transformed point
        Mat3 Jres_P;
        Jres_P << 1 - (diff[0] * diff[0] / bottom), -(diff[0] * diff[1] / bottom), -(diff[0] * diff[2] / bottom),
                -(diff[1] * diff[0] / bottom), 1 - (diff[1] * diff[1] / bottom), -(diff[1] * diff[2] / bottom),
                -(diff[2] * diff[0] / bottom), -(diff[2] * diff[1] / bottom), 1 - (diff[2] * diff[2] / bottom);

        // Jacobian of transformed point wrt the overparameterized transform
        Eigen::Matrix<double, 3, 12> JP_T;
        JP_T << this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1, 0, 0, //
                0, this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1, 0, //
                0, 0, this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1;

        Eigen::Matrix<double, 3, 12> Jr_T;

        // Have to apply the "lift" jacobian to the Interpolation Jacobian because
        // of Ceres local parameterization
        auto J_int = Transformation::Jinterpolated(twist, *(this->scale));
        Eigen::MatrixXd J_lift = interpolated.J_lift();

        Eigen::MatrixXd J_lift_full = transform.J_lift();
        Eigen::MatrixXd pseudo = J_lift_full.completeOrthogonalDecomposition().pseudoInverse();

        Jr_T = this->weight_matrix * Jres_P * JP_T * J_lift * J_int * pseudo;

        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>>(jacobians[0], 3, 12) = Jr_T;
    }

    return true;
}

}  // namespace wave
