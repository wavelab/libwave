#include "wave/optimization/ceres/tiny/robust_line_fit.hpp"

namespace wave {

namespace {

inline void getbPRotation(const Vec3 &normal, Eigen::Matrix<double, 3, 2>& R_reduced) {
    double s2_scaling = 1.0 / (1.0 + normal(2));
    R_reduced(0,0) = 1 - s2_scaling * normal(0) * normal(0);
    R_reduced(0,1) = - s2_scaling * normal(0) * normal(1);
    R_reduced(1,0) = R_reduced(0,1);
    R_reduced(1,1) = 1 - s2_scaling * normal(1) * normal(1);
    R_reduced(2,0) = -normal(0);
    R_reduced(2,1) = -normal(1);
}

inline void boxPlus(const double *global_param, const double *local_param, double *updated_global_param, double *bp_jacobian = nullptr) {
    Eigen::Map<const Vec3> normal(global_param);
    Eigen::Map<const Vec3> pt0(global_param + 3);
    Eigen::Map<const Vec2> Dnormal(local_param);
    Eigen::Map<const Vec2> Dpt0(local_param + 2);

    Eigen::Map<Vec3> normalpD(updated_global_param);
    Eigen::Map<Vec3> pt0pD(updated_global_param + 3);

    Eigen::Matrix<double, 3, 2> R_reduced;
    getbPRotation(normal, R_reduced);

    Vec3 v = R_reduced * Dnormal;

    double a_mag = std::sqrt(Dnormal(0) * Dnormal(0) + Dnormal(1) * Dnormal(1));

    if (a_mag < 1e-5) {
        normalpD.noalias() = normal + v;
    } else {
        normalpD.noalias() = std::cos(a_mag) * normal + std::sin(a_mag) * (v / a_mag);
    }

    pt0pD.noalias() = pt0 + R_reduced * Dpt0;

    if (pt0pD(2) < 0) {
        pt0pD *= -1.0;
    }
    if (bp_jacobian) {
        Eigen::Map<Eigen::Matrix<double, 6, 4>> jacobian(bp_jacobian);

        jacobian.setZero();

        getbPRotation(normalpD, R_reduced);

        jacobian.block<3, 2>(0,0).noalias() = R_reduced;
        jacobian.block<3, 2>(3,2).noalias() = R_reduced;
    }
}

inline double getRotatedErrorAndJacobian(const Vec3 &error, Eigen::Matrix<double, 1, 3> *J) {
    Vec3 ed;
    double norm = error.norm();
    // no need to rotate anything if there is zero error
    const double tolerance = 1e-8;
    if (norm < tolerance) {
        ed << 1.0, 0.0, 0.0;
    } else {
        ed.noalias() = error / norm;
    }
    bool flipped = false;
    if (ed(0) < 0) {
        ed *= -1.0;
        flipped = true;
    }

    Eigen::Matrix<double, 1, 3> rotation;

    double scaling = 1.0 + ed(0);

    rotation(0) = 1.0 - (ed(1)*ed(1) + ed(2)*ed(2)) / scaling;
    rotation(1) = ed(1);
    rotation(2) = ed(2);

    if (J) {
        auto &del_re_del_e = *J;
        Mat3 del_ed_del_e;
        if (norm < tolerance) {
            del_ed_del_e.setZero();
        } else {
            del_ed_del_e = (1 / norm) * (Mat3::Identity() - ed * ed.transpose());
        }
        Mat3 del_R_del_ed;
        del_R_del_ed.setZero();
        del_R_del_ed(0,0) = (ed(1)*ed(1) + ed(2)*ed(2)) / (scaling*scaling);
        del_R_del_ed(0,1) = -2.0 * ed(1) / scaling;
        del_R_del_ed(0,2) = -2.0 * ed(2) / scaling;
        del_R_del_ed(1,1) = 1.0;
        del_R_del_ed(2,2) = 1.0;
        Mat3 del_R_del_e;
        if (flipped) {
            del_R_del_e = -del_R_del_ed * del_ed_del_e;
        } else {
            del_R_del_e = del_R_del_ed * del_ed_del_e;
        }

        del_re_del_e(0) = rotation(0) ;
        del_re_del_e(1) = rotation(1);
        del_re_del_e(2) = rotation(2) ;

//        del_re_del_e(0) = rotation(0) + (del_R_del_e.col(0).transpose() * error)(0);
//        del_re_del_e(1) = rotation(1) + (del_R_del_e.col(1).transpose() * error)(0);
//        del_re_del_e(2) = rotation(2) + (del_R_del_e.col(2).transpose() * error)(0);
    }

    return (rotation * error)(0); // rotated error
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

    if (std::isnan(this->current_line_param(0))) {
        throw std::runtime_error("Bad line param");
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

        Eigen::Matrix<double, 1, 3> J_rotated_error;

        double scaled_error, scaling_gradient, rotated_error;

        if(jacobian) {
            rotated_error = getRotatedErrorAndJacobian(error, &J_rotated_error);
            robustWeight(rotated_error, this->k, scaled_error, &scaling_gradient);
        } else {
            rotated_error = getRotatedErrorAndJacobian(error, nullptr);
            robustWeight(rotated_error, this->k, scaled_error);
        }

        if (std::isnan(rotated_error)) {
            throw std::runtime_error("bad error");
        }
        error_vec(i, 0) = rotated_error;

        if ((std::abs(rotated_error) - error.norm()) > 1e-6) {
            throw std::runtime_error("error disagreement");
        }
        if (jacobian) {
            Eigen::Matrix<double, 3, 6> line_jac;
            // jacobian wrt normal
            line_jac.block<3, 3>(0, 0).noalias() = -normal * diff.transpose() - dp * Mat3::Identity();
            // jacobian wrt pt on line
            line_jac.block<3, 3>(0, 3).noalias() = -Mat3::Identity() + normal * normal.transpose();

            jac_map.block<1, 4>(i, 0).noalias() = J_rotated_error * line_jac * this->cur_lift_jacobian;
        }
    }

    return true;
}

}