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
    int image_width;
    int image_height;
    Mat3 K;
    double hz;

    double dt;
    int frame;

    VOTestCamera()
        : image_width{-1}, image_height{-1}, K{}, hz{-1}, dt{0}, frame{-1} {}

    /** Check whether camera is triggered at this time-step
     * @returns boolean to denote true or false
     */
    bool update(double dt);

    /** Returns a list of observable 3D world features
     *
     * @param dt Update time step
     * @param features 3D world features to test (NWU)
     * @param rpy Vector containing roll, pitch, yaw of camera (EDN)
     * @param t Vector containining position in x, y and z (EDN)
     * @param observed Observed 3D features in the image frame
     */
    int checkFeatures(double dt,
                      const MatX &features,
                      const Vec3 &rpy,
                      const Vec3 &t,
                      std::vector<std::pair<Vec2, Vec3>> &observed);
};

/** A set of feature observations at one timestep */
struct VOTestInstant {
    /** A time in nominal seconds */
    double time;

    /** The corresponding camera frame, or -1 if no camera observations */
    int camera_frame = -1;

    /** Robot pose in x, y, z (NWU) */
    Vec3 robot_pose;

    /** Feature observations where the first of each pair is the measurement in
     * the image frame, and the second is the world feature coordinates.
     */
    std::vector<std::pair<Vec2, Vec3>> observed;
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
    MatX features;

    /** For each time step, a set of measurements */
    std::vector<VOTestInstant> measurements;
};

/**
 * Synthetic VO dataset generator.
 *
 * This class simulates a two wheel robot traversing in circle where the world
 * has 3D random feature points, and stores the data in a VOTestDataset object.
 */
class VOTestDatasetGenerator {
 public:
    VOTestCamera camera;
    int nb_features;
    Vec2 feature_x_bounds;
    Vec2 feature_y_bounds;
    Vec2 feature_z_bounds;

    VOTestDatasetGenerator()
        : camera{},
          nb_features{-1},
          feature_x_bounds{},
          feature_y_bounds{},
          feature_z_bounds{} {}
    explicit VOTestDatasetGenerator(const std::string &config_file);

    /** Generate random 3D features in the world frame
     *
     * @param features Random generated 3D world features, where each column
     * represents a feature and each row represents the feature position in x,
     * y, z (NWU)
     */
    void generateRandom3DFeatures(MatX &features);

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
};


/** Record observed features in camera frame to file in csv format
 *
 * The csv format is as follows:
 *
 * - Time_step
 * - Number of features observed
 * - Features x, y, z (NWU) where has each row representing a feature and
 *   each column representing the feature position in x, y, z (NWU)
 *
 * @param measurements object containing the time, pose, and observations
 * @param output_path output file which will be created
 */
void recordMeasurements(const VOTestInstant &measurements,
                        const std::string &output_path);

/** Record 3D features to file in csv format
 *
 * The csv format has each row representing a feature and each column
 * representing the feature position in x, y, z (NWU)
 *
 * @param features 3D features to save
 * @param output_path output file which will be created
 */
void record3DFeatures(const MatX &features, const std::string &output_path);

/**
 * Records a generated dataset to separate files.
 *
 * @param dataset the dataset object
 * @param output_path destination directory
 *
 * The following files are stored in csv format:
 *
 * - features.dat (see `record3DFeatures`)
 * - state.dat robot position (x, y, angle) at each timestep with a header
 * - observed_*.dat, N files with feature observations (see
 *   `recordMeasurements`)
 * - index.dat, listing the N files above
 *
 * @todo fix the documentation of csv format. For some files the header does not
 * seem to refer the actual contents, etc.
 */
void recordTestData(const VOTestDataset &dataset,
                    const std::string &output_path);


/** @} end of group */
}  // namespace wave
#endif
