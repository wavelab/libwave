#ifndef WAVE_IMPLICIT_LINE_IMPL_HPP
#define WAVE_IMPLICIT_LINE_IMPL_HPP

#include "../implicit_line.hpp"

namespace wave {

template<int cnt, int... idx>
bool ImplicitLineResidual::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, cnt, 1>> error(residuals);
    Eigen::Map<const Vec3> normal(&parameters[0][0]);
    std::vector<Eigen::Map<const Vec12>> states;
    for (int i = 0; i < cnt; i++) {
        states.emplace_back(Eigen::Map<const Vec12>(&parameters[i][0]));
    }

    //Calculate average point
    Vec3 avg;
    for (const auto &pt : this->pts.tpts) {
        avg = avg + pt;
    }
    double cnt_d = static_cast<double>(cnt);
    avg = avg / cnt_d;


    if (jacobians) {
        std::vector<Eigen::Map<Eigen::Matrix<double, cnt, 12, Eigen::RowMajor>>> state_jacs;
        Eigen::Map<Eigen::Matrix<double, cnt, 3, Eigen::RowMajor>> normal_jac(jacobians[0]);
        normal_jac.template setZero();
        for (uint32_t i = 0; i < sizeof...(idx); i++) {
            state_jacs.emplace_back(Eigen::Map<Eigen::Matrix<double, cnt, 12, Eigen::RowMajor>>(jacobians[i+1]));
            state_jacs.at(i).template setZero();
        }
        // Calculate error and jacobians
        for (uint32_t i = 0; i < this->pts.tpts.size(); i++) {
            auto diff = this->pts.tpts.at(i) - avg;
            double scalar = (normal.transpose() * diff).sqrt();
            double nm1on = (cnt_d - 1.0) / cnt_d;
            double oon = 1.0 / cnt_d;
            error.template block<3, 1>(i * 3, 0) = diff - scalar * normal;
            normal_jac.template block<3, 3>(i*3, 0) = -normal * diff.transpose() / (2*scalar) - scalar * Mat3::Identity();
            Eigen::Matrix<double, 3, 3> del_e_del_diff;
            del_e_del_diff = Mat3::Identity() - normal * normal.transpose() / (2*scalar);
            state_jacs.at(this->pts.p_states.at(i)).template block<3, 12>(i*3, 0) += del_e_del_diff * nm1on * (*(this->pts.prev_jac.at(i)));
            state_jacs.at(this->pts.n_states.at(i)).template block<3, 12>(i*3, 0) += del_e_del_diff * nm1on * (*(this->pts.next_jac.at(i)));
            for (uint32_t j = 0; j <this->pts.tpts.size(); j++) {
                if (i == j) {
                    continue;
                }
                state_jacs.at(this->pts.p_states.at(j)).template block<3, 12>(i*3, 0) -= del_e_del_diff * oon * (*(this->pts.prev_jac.at(j)));
                state_jacs.at(this->pts.n_states.at(j)).template block<3, 12>(i*3, 0) -= del_e_del_diff * oon * (*(this->pts.next_jac.at(j)));
            }
        }
    } else {
        // Calculate error
        for (uint32_t i = 0; i < this->pts.tpts.size(); i++) {
            auto diff = this->pts.tpts.at(i) - avg;
            error.template block<3, 1>(i * 3, 0) = diff - (normal.transpose() * diff).sqrt() * normal;
        }
    }
    return true;
}

}

#endif //WAVE_IMPLICIT_LINE_IMPL_HPP
