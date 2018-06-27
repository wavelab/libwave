#include "wave/odometry/odometry_callback.hpp"

namespace wave {

void OdometryCallback::PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) {
    if (new_evaluation_point) {
        this->transformer->update(*(this->traj), *(this->traj_stamps));
        const unsigned long stop = this->feat_pts->size() * this->feat_pts->front().size();
        const unsigned long feat_cnt = this->feat_pts->front().size();
        for (uint32_t i = 0; i < stop; ++i){
            auto scan_idx = static_cast<uint32_t>(i / feat_cnt);
            auto feat_idx = static_cast<uint32_t>(i % feat_cnt);
            this->transformer->transformToStart(this->feat_pts->[scan_idx][feat_idx],
                                                this->feat_ptsT->[scan_idx][feat_idx]);
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
        this->pose_diff.at(i) = this->traj->at(i+1).pose.manifoldMinus(this->traj->at(i).pose);
        this->J_logmaps.at(i) = Transformation<>::SE3ApproxInvLeftJacobian(this->pose_diff.at(i));
    }
#pragma omp parallel for
    for ()
}

}
