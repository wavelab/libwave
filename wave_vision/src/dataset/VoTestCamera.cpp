#include "wave/vision/dataset/VoTestCamera.hpp"

namespace wave {


bool VoTestCamera::update(double dt) {
    this->dt += dt;

    if (this->dt > (1.0 / this->hz)) {
        this->dt = 0.0;
        this->frame++;
        return true;
    }

    return false;
}

int VoTestCamera::observeLandmarks(double dt,
                                   const LandmarkMap &landmarks,
                                   const Quaternion &q_GC,
                                   const Vec3 &G_p_GC,
                                   std::vector<LandmarkObservation> &observed) {
    // pre-check
    if (this->update(dt) == false) {
        return 1;
    }

    // get rotation matrix from world frame to camera frame
    Mat3 R_GC = q_GC.matrix();

    // check which landmarks in 3d are observable from camera
    observed.clear();
    for (auto landmark : landmarks) {
        const auto &point = landmark.second;

        // project 3D world point to 2D image plane, also checking cheirality
        Vec2 meas;
        auto in_front = pinholeProject(this->K, R_GC, G_p_GC, point, meas);
        if (in_front) {
            // check to see if feature observed is within image plane
            if (meas.x() < this->image_width && meas.x() > 0 &&
                meas.y() < this->image_height && meas.y() > 0) {
                observed.emplace_back(landmark.first, meas);
            }
        }
    }
    return 0;
}

}  // namespace wave
