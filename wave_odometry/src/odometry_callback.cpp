#include "wave/odometry/odometry_callback.hpp"

namespace wave {

void OdometryCallback::PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) {
    if (new_evaluation_point) {
        this->transformer->update(trajectory, stamps);
        this->transformer->transformToStart(this->feat_pts, this->feat_ptsT);
        this->old_jacobians = true;
    }
    if (evaluate_jacobians) {
        if(this->old_jacobians) {
            this->evaluateJacobians();
            this->old_jacobians = false;
        }
    }
}

void OdometryCallback::evaluateJacobians() {

}

}
