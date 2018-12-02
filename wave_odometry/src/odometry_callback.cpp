#include "wave/odometry/odometry_callback.hpp"

namespace wave {

OdometryCallback::OdometryCallback(const Vec<VecE<Eigen::Tensor<float, 2>>> *feat_pts,
                                   const VecE<Eigen::Tensor<float, 2>> *prev_feat_pts,
                                   const VecE<Eigen::Tensor<float, 2>> *cur_feat_pts,
                                   Vec<VecE<MatX>> *feat_ptsT,
                                   VecE<MatX> *prev_feat_ptsT,
                                   VecE<MatX> *cur_feat_ptsT,
                                   const VecE<PoseVel> *traj,
                                   Vec<Vec<VecE<MatX>>> *ptT_jacobians,
                                   Vec<Vec<float>> *jac_stamps,
                                   const Vec<float> *traj_stamps,
                                   const Vec<float> *scan_stamps,
                                   Transformer *transformer)
    : ceres::EvaluationCallback(),
      feat_pts(feat_pts),
      prev_feat_pts(prev_feat_pts),
      cur_feat_pts(cur_feat_pts),
      feat_ptsT(feat_ptsT),
      prev_feat_ptsT(prev_feat_ptsT),
      cur_feat_ptsT(cur_feat_ptsT),
      traj(traj),
      jacobians(ptT_jacobians),
      jac_stamps(jac_stamps),
      traj_stamps(traj_stamps),
      scan_stamps(scan_stamps),
      transformer(transformer) {
    if (this->prev_feat_pts && this->cur_feat_pts) {
        for (uint32_t feat_id = 0; feat_id < this->prev_feat_pts->size(); ++feat_id) {
            this->prev_feat_ptsT->at(feat_id).resize(3, this->prev_feat_pts->at(feat_id).dimension(1));
            this->cur_feat_ptsT->at(feat_id).resize(3, this->cur_feat_pts->at(feat_id).dimension(1));
            for (uint32_t scan_id = 0; scan_id < this->feat_pts->size(); ++scan_id) {
                this->feat_ptsT->at(scan_id).at(feat_id).resize(3, this->feat_pts->at(scan_id).at(feat_id).dimension(1));
            }
        }
    }

    this->pose_diff.resize(traj->size() - 1);
    this->Pose_diff.resize(traj->size() - 1);
    this->J_logmaps.resize(traj->size() - 1);

    this->jac_stamps->resize(this->traj_stamps->size() - 1);
    this->jacobians->resize(this->traj_stamps->size() - 1);

    for (uint32_t gap_index = 0; gap_index + 1 < this->traj_stamps->size(); ++gap_index) {
        float delta_t = this->traj_stamps->at(gap_index + 1) - this->traj_stamps->at(gap_index);
        auto steps = static_cast<uint32_t>(delta_t / 0.01f);
        float step_time = delta_t / steps;
        this->jac_stamps->at(gap_index).clear();
        this->jac_stamps->at(gap_index).emplace_back(this->traj_stamps->at(gap_index));
        for (uint32_t i = 1; i < steps; ++i) {
            this->jac_stamps->at(gap_index).emplace_back(step_time * i + this->traj_stamps->at(gap_index));
        }
        this->jac_stamps->at(gap_index).emplace_back(this->traj_stamps->at(gap_index + 1));
        this->jacobians->at(gap_index).resize(4);
        for (auto &jacs : this->jacobians->at(gap_index)) {
            jacs.resize(steps + 1);
        }
    }
    this->PrepareForEvaluation(false, true);
}

void OdometryCallback::PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) {
    if (new_evaluation_point) {
        this->transformer->update(*(this->traj), *(this->traj_stamps));
        for (uint32_t j = 0; j < this->feat_pts->at(0).size(); ++j) {
            for (uint32_t i = 0; i < this->feat_pts->size(); ++i) {
                this->transformer->transformToStart(this->feat_pts->at(i)[j], this->feat_ptsT->at(i)[j], i);
            }
            if (this->prev_feat_pts && this->cur_feat_pts) {
                auto last_scan_id = this->feat_pts->size() - 1;
                this->transformer->transformToStart(this->cur_feat_pts->at(j), this->cur_feat_ptsT->at(j), last_scan_id);
                this->transformer->transformToStart(this->prev_feat_pts->at(j), this->prev_feat_ptsT->at(j), --last_scan_id);
            }
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
        this->Pose_diff.at(i).setFromExpMap(this->pose_diff.at(i));
        this->J_logmaps.at(i) = Transformation<>::SE3ApproxInvLeftJacobian(this->pose_diff.at(i));
    }

    // calculate a grid of jacobians for interpolation within each residual later
    for (uint32_t gap_index = 0; gap_index + 1 < this->traj_stamps->size(); ++gap_index) {
        float delta_t = this->traj_stamps->at(gap_index + 1) - this->traj_stamps->at(gap_index);
        for (uint32_t t_idx = 0; t_idx < this->jac_stamps->at(gap_index).size(); ++t_idx) {
            const auto &time = this->jac_stamps->at(gap_index).at(t_idx);
            float T1 = time - this->traj_stamps->at(gap_index);
            float T2 = this->traj_stamps->at(gap_index + 1) - time;
            float invT = 1.0f / delta_t;

            Vec3 interp;

            // candle(0,0) and (0,1)
            interp(1) = (T1 * T1 * (4 * T1 - 3 * delta_t + 6 * T2)) * invT * invT * invT;
            interp(2) = -(T1 * T1 * (2 * T1 - 2 * delta_t + 3 * T2)) * invT * invT;
            // candle
            interp(0) = T1 - delta_t * interp(1) - interp(2);

            Vec6 inc_twist = interp(0) * this->traj->at(gap_index).vel + interp(1) * this->pose_diff.at(gap_index) +
                             interp(2) * this->J_logmaps.at(gap_index) * this->traj->at(gap_index + 1).vel;

            Mat6 J_inc_twist, Ad_inc_twist, Ad_twist;
            Transformation<>::SE3ApproxLeftJacobian(inc_twist, J_inc_twist);
            Ad_inc_twist = Transformation<>::expMapAdjoint(inc_twist);
            Ad_twist = this->Pose_diff.at(gap_index).adjointRep();

            /// jacobian for first pose
            this->jacobians->at(gap_index).at(0).at(t_idx).noalias() =
              Ad_inc_twist - interp(1) * J_inc_twist * this->J_logmaps.at(gap_index) * Ad_twist;
            /// jacobian for second pose
            this->jacobians->at(gap_index).at(2).at(t_idx).noalias() = interp(1) * J_inc_twist * this->J_logmaps.at(gap_index);
            /// jacobian for first twist

            this->jacobians->at(gap_index).at(1).at(t_idx).noalias() = interp(0) * J_inc_twist;
            /// jacobian for second twist
            this->jacobians->at(gap_index).at(3).at(t_idx).noalias() = interp(2) * J_inc_twist * this->J_logmaps.at(gap_index);
        }
    }
}
}
