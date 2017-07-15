/**
 * @file
 * VO Synthetic Dataset Generator
 * class.
 * @ingroup vision
 */
#ifndef WAVE_VISION_DATASET_HPP
#define WAVE_VISION_DATASET_HPP

#include <sys/stat.h>
#include <sys/types.h>

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
                       const LandmarkMap &landmarks,
                       const Vec3 &rpy,
                       const Vec3 &t,
                       std::vector<LandmarkObservation> &observed);
};


/** A set of feature observations at one timestep */
struct VOTestInstant {
    /** A time in nominal seconds */
    double time;

    /** The corresponding camera frame, or -1 if no camera observations */
    int camera_frame = -1;

    /** True robot pose in x, y, z (NWU) */
    Vec3 robot_pose;

    /** Feature observations where the first of each pair is the measurement in
     * the image frame, and the second is the landmark id */
    std::vector<LandmarkObservation> features_observed;
};

/**
 * A set of generated dataset data.
 *
 * This includes:
 * - Robot pose (robot ground truth)
 * - 3D Features in the world (landmark ground truth)
 * - 3D Features observed in image frame on the two wheel robot
 */
struct VOTestDataset {
    /** Ground truth 3D world features, where each column represents a feature
     * and each row represents the feature position in x, y, z (NWU)  */
    LandmarkMap landmarks;

    /** For each time step, a set of measurements */
    std::vector<VOTestInstant> states;

    int outputToFile(const std::string &output_dir);
    int outputLandmarks(const std::string &output_path);
    int outputObserved(const std::string &output_path);
    int outputRobotState(const std::string &output_dir);
};

/**
 * Synthetic VO dataset generator.
 *
 * This class simulates a two wheel robot traversing in circle where the world
 * has 3D random feature points, and stores the data in a VOTestDataset object.
 */
class VOTestDatasetGenerator {
 public:
    int configure(const std::string &config_file);

    /** Generate random 3D landmarks in the world frame
     *
     * @returns  matrix where each column represents a landmark and each row
     * represents the feature position in x, y, z
     */
    LandmarkMap generateLandmarks();

    /** Simulates a two wheel robot traversing in a world with random 3D feature
     * points, and records:
     *
     * - Robot pose (robot ground truth)
     * - 3D Features in the world (landmark ground truth)
     * - 3D Features observed in image frame on the two wheel robot
     *
     * in a VOTestDataset object.
     *
     * A measurement including robot pose is stored at every timestep (with an
     * arbitrary dt), but feature observations are only made at some timesteps.
     */
    VOTestDataset generate();

    bool configured = false;

    VOTestCamera camera;
    int nb_landmarks = 0;
    Vec2 landmark_x_bounds = Vec2::Zero();
    Vec2 landmark_y_bounds = Vec2::Zero();
    Vec2 landmark_z_bounds = Vec2::Zero();
};

/** @} end of group */
}  // namespace wave
#endif
