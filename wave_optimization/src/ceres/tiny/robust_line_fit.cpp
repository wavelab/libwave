#include "wave/optimization/ceres/tiny/robust_line_fit.hpp"

namespace wave {

namespace {

inline void boxPlus(const double *global_param, const double *local_param, double *updated_global_param, double *bp_jacobian = nullptr) {
    Eigen::Map<const Vec3> normal(global_param);
    Eigen::Map<const Vec3> pt0(global_param + 3);
    Eigen::Map<const Vec2> Dnormal(local_param);
    Eigen::Map<const Vec2> Dpt0(local_param + 2);

    Eigen::Map<Vec3> normalpD(updated_global_param);
    Eigen::Map<Vec3> pt0pD(updated_global_param + 3);

    // Cross product of unit z vector with p
    Vec2 axis;
    axis << -normal(1), normal(0); //, 0.0;

    double c = normal(2);

    Mat3 skew = Mat3::Zero();
    skew(0, 2) = axis(1);
    skew(1, 2) = -axis(0);
    skew(2, 0) = -axis(1);
    skew(2, 1) = axis(0);

    Mat3 R = Mat3::Identity() + skew + (1.0 / (1.0 + c)) * skew * skew;

    Vec3 v = R.block<3, 2>(0, 0) * Dnormal;

    double a_mag = std::sqrt(Dnormal(0) * Dnormal(0) + Dnormal(1) * Dnormal(1));

    if (a_mag < 1e-5) {
        normalpD.noalias() = normal + v;
    } else {
        normalpD.noalias() = std::cos(a_mag) * normal + std::sin(a_mag) * (v / a_mag);
    }

    pt0pD.noalias() = pt0 + R.block<3, 2>(0,0) * Dpt0;

    if (pt0pD(2) < 0) {
        pt0pD *= -1.0;
    }
    if (bp_jacobian) {
        Eigen::Map<Eigen::Matrix<double, 6, 4>> jacobian(bp_jacobian);

        jacobian.block<3, 2>(0,0).noalias() = R.block<3, 2>(0,0);
        jacobian.block<3, 2>(3,2).noalias() = R.block<3, 2>(0,0);
    }
}

inline Eigen::Matrix<double, 1, 3> getRotation(const Vec3 &error) {
    Vec3 ed = error;
    ed.normalize();
    if (ed(0) < 0) {
        ed *= -1.0;
    }

    Eigen::Matrix<double, 1, 3> retval;

    retval(0) = 1.0 - (ed(1)*ed(1) + ed(2)*ed(2))/(1.0 + ed(0));
    retval(1) = ed(1);
    retval(2) = ed(2);

    return retval;
}

void robustWeight(const double &error, const double &k, double &scaled_error, double *scaling_gradient = nullptr) {
    // outlier region
    if (error > k) {
        scaled_error = k / std::sqrt(6.0);
        if (scaling_gradient) {
            *scaling_gradient = 0.0;
        }
    } else if (error < -k) {
        scaled_error = -k / std::sqrt(6.0);
        if (scaling_gradient) {
            *scaling_gradient = 0.0;
        }
    } else {
        // inlier region
        double x2 = error * error;
        double k2 = k * k;
        double val = std::sqrt(x2 * x2 - 3.0 * k2 * x2 + 3.0 * k2 * k2);
        scaled_error = (1.0 / (k2 * sqrt(6.0))) * error * val;
        if (scaling_gradient) {
            *scaling_gradient = (1.0 / (k2 * sqrt(6.0))) * (val + (2.0 * x2 * x2 - 3.0 * x2 * k2) / val);
        }
    }
}

}

bool RobustLineFit::operator()(const double *parameters, double *residuals, double *jacobian) const {
    // If the solver is asking for the jacobian, it means that the step has been accepted, so update global parameterization
    // and zero-out the local paramterization
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, NUM_PARAMETERS>> jac_map(nullptr, 1, NUM_PARAMETERS);
    if (jacobian) {
        boxPlus(this->line_param, parameters, this->current_line_param.data(), this->cur_lift_jacobian.data());
        for (uint32_t i = 0; i < NUM_PARAMETERS; ++i) {
            this->opt_param[i] = 0;
        }
        Eigen::Map<Vec6> global_param(this->line_param);
        global_param.noalias() = this->current_line_param;
        // This does not actually allocate memory
        new (&jac_map) Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, NUM_PARAMETERS>>(jacobian, this->pts.size(), NUM_PARAMETERS);
    } else {
        boxPlus(this->line_param, parameters, this->current_line_param.data());
    }

    auto normal = this->current_line_param.block<3,1>(0,0);
    auto pt0 = this->current_line_param.block<3,1>(3,0);
    Vec3 diff;

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>> error_vec(residuals, this->pts.size(), 1);
    for (uint32_t i = 0; i < this->pts.size(); ++i) {
        Eigen::Map<const Vec3f> cur_pt(this->pts.at(i));

        diff = cur_pt.cast<double>() - pt0;
        double dp = (normal.transpose() * diff)(0);
        Vec3 error = diff - dp * normal;

        Eigen::Matrix<double, 1, 3> R = getRotation(error);
        double rotated_error = (R * error)(0);

        double scaled_error, scaling_gradient;

        if(jacobian) {
            robustWeight(rotated_error, this->k, scaled_error, &scaling_gradient);
        } else {
            robustWeight(rotated_error, this->k, scaled_error);
        }

//        error_vec(i, 0) = scaled_error;
        error_vec(i, 0) = rotated_error;
        if (jacobian) {
            Eigen::Matrix<double, 3, 6> line_jac;
            // jacobian wrt normal
            line_jac.block<3, 3>(0, 0).noalias() = -normal * diff.transpose() - dp * Mat3::Identity();
            // jacobian wrt pt on line
            line_jac.block<3, 3>(0, 3).noalias() = -Mat3::Identity() + normal * normal.transpose();

//            jac_map.block<1, 4>(i, 0).noalias() = scaling_gradient * R * line_jac * this->cur_lift_jacobian;
            jac_map.block<1, 4>(i, 0).noalias() = R * line_jac * this->cur_lift_jacobian;
        }
    }

    return true;
}

}