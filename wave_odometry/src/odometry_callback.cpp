#include "wave/odometry/odometry_callback.hpp"

namespace wave {

OdometryCallback::OdometryCallback(const Vec<VecE<Eigen::Tensor<float, 2>>> *feat_pts,
                                   Vec<VecE<Eigen::Tensor<float, 2>>> *feat_ptsT,
                                   const VecE<PoseVel> *traj,
                                   const Vec<float> *traj_stamps,
                                   Transformer *transformer)
    : ceres::EvaluationCallback(),
      feat_pts(feat_pts),
      feat_ptsT(feat_ptsT),
      traj(traj),
      traj_stamps(traj_stamps),
      transformer(transformer) {

    Eigen::TensorMap<Eigen::Tensor<const float, 1>> T_times(this->traj_stamps->data(), this->traj_stamps->size());
    for(uint32_t scan_idx = 0; scan_idx < this->feat_pts->size(); ++scan_idx) {
        for(uint32_t feat_idx = 0; feat_idx < this->feat_pts->at(scan_idx).size(); ++feat_idx) {
            //todo figure out how to vectorize this using horrible Tensor interface
            const auto &pts = this->feat_pts->at(scan_idx).at(feat_idx);
            auto &interp = this->interp_factors.at(scan_idx).at(feat_idx);
            uint32_t stamp_idx = 0;
            for(uint32_t pt_idx = 0; pt_idx < pts.dimension(1); ++pt_idx) {
                float tau = pts(3, pt_idx);
                if (tau > this->traj_stamps->at(stamp_idx + 1)) {
                    ++stamp_idx;
                }
                float T1 = tau - this->traj_stamps->at(stamp_idx);
                float T2 = this->traj_stamps->at(stamp_idx + 1) - tau;
                float dT = this->traj_stamps->at(stamp_idx + 1) - this->traj_stamps->at(stamp_idx);
                float invT = 1.0f / dT;

                interp(0, pt_idx) = (T1 * T1 * (4 * T1 - 3 * dT + 6 * T2)) * invT * invT * invT;
                float temp = -(T1 * T1 * (2 * T1 - 2 * dT + 3 * T2)) * invT * invT;

                interp(1, pt_idx) = 1.0f - interp(0, pt_idx);
                interp(2, pt_idx) = T1 - dT * interp(0, pt_idx) - temp;
            }
        }
    }
}

void OdometryCallback::PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) {
    if (new_evaluation_point) {
        this->transformer->update(*(this->traj), *(this->traj_stamps));
        const unsigned long stop = this->feat_pts->size() * this->feat_pts->front().size();
        const unsigned long feat_cnt = this->feat_pts->front().size();
        for (uint32_t i = 0; i < stop; ++i) {
            auto scan_idx = static_cast<uint32_t>(i / feat_cnt);
            auto feat_idx = static_cast<uint32_t>(i % feat_cnt);
            this->transformer->transformToStart(this->feat_pts->at(scan_idx)[feat_idx],
                                                this->feat_ptsT->at(scan_idx)[feat_idx]);
        }
        this->old_jacobians = true;
    }
    if (evaluate_jacobians && this->old_jacobians) {
        this->evaluateJacobians();
        this->old_jacobians = false;
    }
}

void OdometryCallback::evaluateJacobians() {
    // First calculate the difference, and logmap jacobian between all the transforms
    for (uint32_t i = 0; i + 1 < this->traj->size(); ++i) {
        this->pose_diff.at(i) = this->traj->at(i + 1).pose.manifoldMinus(this->traj->at(i).pose);
        this->J_logmaps.at(i) = Transformation<>::SE3ApproxInvLeftJacobian(this->pose_diff.at(i));
    }

#pragma omp parallel for
    for (uint32_t scan_idx = 0; scan_idx < this->feat_pts->size(); ++scan_idx) {
        for (uint32_t feat_idx = 0; feat_idx < this->feat_pts->at(scan_idx).size(); ++feat_idx) {
            for (uint32_t pt_cnt = 0; pt_cnt < this->feat_pts->at(scan_idx).at(feat_idx).dimension(1); ++pt_cnt) {
                auto &intfac = this->interp_factors.at(scan_idx).at(feat_idx);
                auto &pt_jacs = this->ptT_jacobians->at(scan_idx).at(feat_idx);
                const auto &pts = this->feat_pts->at(scan_idx).at(feat_idx);

                /// need to find the state indices based on point timestamps
                auto iter = std::lower_bound(this->traj_stamps->begin(), this->traj_stamps->end(), pts(3, pt_cnt));
                auto prev_idx = static_cast<uint32_t>(iter - this->traj_stamps->begin());
                uint32_t next_idx = prev_idx + 1;

                Vec6 inc_twist = intfac(0, pt_cnt) * this->traj->at(prev_idx).vel +
                                 intfac(1, pt_cnt) * this->pose_diff.at(prev_idx) +
                                 intfac(2, pt_cnt) * this->J_logmaps.at(prev_idx) * this->traj->at(next_idx).vel;

                Mat6 J_inc_twist, Ad_inc_twist, Ad_twist;
                Transformation<>::SE3ApproxLeftJacobian(inc_twist, J_inc_twist);
                Transformation<>::skewSymmetric6(inc_twist, Ad_inc_twist);
                Transformation<>::skewSymmetric6(this->pose_diff.at(prev_idx), Ad_twist);

                MatX jacobian(6, 12);
                jacobian.block<6, 6>(0, 6).setZero();
                // column major storage so this is fine.
                Eigen::TensorMap<Eigen::Tensor<double, 2>> t6_12map(jacobian.data(), 6, 12);
                Eigen::TensorMap<Eigen::Tensor<double, 2>> t6_6map(jacobian.data(), 6, 6);

                /// jacobian for first pose
                jacobian.block<6, 6>(0, 0) =
                  Ad_inc_twist - intfac(1, pt_cnt) * J_inc_twist * this->J_logmaps.at(prev_idx) * Ad_twist;
                pt_jacs.at(0).chip(pt_cnt, 0) = t6_12map;
                /// jacobian for first twist
                jacobian.block<6, 6>(0, 0) = intfac(0, pt_cnt) * J_inc_twist;
                pt_jacs.at(1).chip(pt_cnt, 0) = t6_6map;
                /// jacobian for second pose
                jacobian.block<6, 6>(0, 0) = intfac(1, pt_cnt) * J_inc_twist * this->J_logmaps.at(prev_idx);
                pt_jacs.at(2).chip(pt_cnt, 0) = t6_12map;
                /// jacobian for second twist
                jacobian.block<6, 6>(0, 0) = intfac(2, pt_cnt) * J_inc_twist * this->J_logmaps.at(prev_idx);
            }
        }
    }
}

}
