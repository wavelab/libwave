/**
 * @file
 * VO Synthetic Dataset Generator
 * class.
 * @ingroup vision
 */
#ifndef WAVE_VISION_DATASET_HPP
#define WAVE_VISION_DATASET_HPP

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"
#include "wave/kinematics/two_wheel.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

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
class VOTestCamera {
 public:
    int image_width = 0;
    int image_height = 0;
    Mat3 K = Mat3::Identity();
    double hz = 0.0;

    double dt = 0.0;
    int frame = 0;

    VOTestCamera() {}

    VOTestCamera(int image_width, int image_height, Mat3 K, double hz)
        : image_width{image_width}, image_height{image_height}, K{K}, hz{hz} {}

    /** Check whether camera is triggered at this time-step
     * @returns boolean to denote true or false
     */
    bool update(double dt);

    /** Returns a list of observable 3D world landmarks
     *
     * @param dt Update time step
     * @param features 3D world features to test (NWU)
     * @param rpy Vector containing roll, pitch, yaw of camera (EDN)
     * @param t Vector containining position in x, y and z (EDN)
     * @param observed Observed 3D features in the image frame
     */
    int checkLandmarks(double dt,
                       const std::map<Vec3, int, VecComparator> &landmarks,
                       const Vec3 &rpy,
                       const Vec3 &t,
                       std::vector<std::pair<Vec2, int>> &observed);
};

class VOTestDataset {
 public:
    bool configured = false;

    VOTestCamera camera;
    int nb_landmarks = 0;
    Vec2 landmark_x_bounds = Vec2::Zero();
    Vec2 landmark_y_bounds = Vec2::Zero();
    Vec2 landmark_z_bounds = Vec2::Zero();

    std::map<Vec3, int, VecComparator> landmarks;
    std::vector<std::pair<double, Vec3>> robot_state;
    std::vector<std::vector<std::pair<Vec2, int>>> features_observed;

    VOTestDataset() {}

    int configure(const std::string &config_file);
    int generateLandmarks();
    int outputLandmarks(const std::string &output_path);
    int outputObserved(const std::string &output_path);
    int outputRobotState(const std::string &output_dir);
    int simulateVODataset();
    int generateTestData(const std::string &output_path);
};

/** @} end of group */
}  // namespace wave
#endif
