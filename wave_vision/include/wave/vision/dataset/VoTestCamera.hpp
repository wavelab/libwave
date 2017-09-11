/**
 * @file
 * @ingroup vision
 */
#ifndef WAVE_VISION_VOTESTCAMERA_HPP
#define WAVE_VISION_VOTESTCAMERA_HPP

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"
#include "wave/kinematics/two_wheel.hpp"
#include "wave/containers/landmark_measurement.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Container for landmarks sorted by id */
using LandmarkMap = std::map<LandmarkId, Vec3>;

/** One observation of a landmark, in pixel coordinates */
using LandmarkObservation = std::pair<LandmarkId, Vec2>;

/** VO Test Camera that mimicks a real camera, specifically it contains:
 *
 * - Image width
 * - Image height
 * - Camera intrinsics matrix
 * - Camera update rate (Hz)
 * - Frame number
 *
 *  This class specifically is used to check whether 3D world features is
 *  observable by the camera in a certain position and orientation.
 */
class VoTestCamera {
 public:
    int image_width = 0;
    int image_height = 0;
    Mat3 K = Mat3::Identity();
    double hz = 0.0;

    double dt = 0.0;
    int frame = 0;

    VoTestCamera() {}

    VoTestCamera(int image_width, int image_height, Mat3 K, double hz)
        : image_width{image_width}, image_height{image_height}, K{K}, hz{hz} {}

    /** Check whether camera is triggered at this time-step
     * @returns boolean to denote true or false
     */
    bool update(double dt);

    /** Gives a list of measurements of visible landmarks, if enough time has
     * passed since the last frame
     *
     * @param dt Update time step
     * @param landmarks map of landmarks with 3D position in world frame
     * @param q_GC orientation of camera in global frame
     * @param G_p_GC translation to camera from origin, in global frame
     * @param observed Observed 3D features in the image frame
     * @returns 0 if observations were made, 1 if not enough time passed
     */
    int observeLandmarks(double dt,
                         const LandmarkMap &landmarks,
                         const Quaternion &q_GC,
                         const Vec3 &G_p_GC,
                         std::vector<LandmarkObservation> &observed);
};


/** @} end of group */
}  // namespace wave
#endif  // WAVE_VISION_VOTESTCAMERA_HPP
