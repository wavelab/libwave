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
     * @param features 3D world features to test
     * @param rpy Vector containing roll, pitch, yaw of camera
     * @param t Vector containining position in x, y and z in (NWU)
     * @param observed Observed 3D features in the image frame
     */
    int checkFeatures(double dt,
                      const MatX &features,
                      const Vec3 &rpy,
                      const Vec3 &t,
                      std::vector<std::pair<Vec2, Vec3>> &observed);
};

/**
 * Synthetic VO dataset generator.
 *
 * This class simulates a two wheel robot traversing in circle where the world
 * has 3D random feature points, and records:
 *
 * - Robot pose (robot ground truth)
 * - 3D Features in the world (landmark ground truth)
 * - 3D Features observed in image frame on the two wheel robot
 *
 * in separate files and in a dedicated folder.
 */
class VOTestDataset {
 public:
    VOTestCamera camera;
    int nb_features;
    Vec2 feature_x_bounds;
    Vec2 feature_y_bounds;
    Vec2 feature_z_bounds;

    VOTestDataset()
        : camera{},
          nb_features{-1},
          feature_x_bounds{},
          feature_y_bounds{},
          feature_z_bounds{} {}
    explicit VOTestDataset(const std::string &config_file);

    /** Generate random 3D features in the world frame
     *
     * @param features Random generated 3D world features, where each column
     * represents a feature and each row represents the feature position in x,
     * y, z (NWU)
     */
    void generateRandom3DFeatures(MatX &features);

    /** Record 3D features to file in csv format
     *
     * The csv format has each row representing a feature and each column
     * representing the feature position in x, y, z (NWU)
     *
     * @param output_path Output path to save 3D features, the resulting file
     * will be in `<output_path>/features.dat`
     * @param features 3D features to save
     */
    void record3DFeatures(const std::string &output_path, const MatX &features);

    /** Record observed features in camera frame to file in csv format
     *
     * The csv format is as follows:
     *
     * - Time_step
     * - Number of features observed
     * - Features x, y, z (NWU) where has each row representing a feature and
     *   each column representing the feature position in x, y, z (NWU)
     *
     * @param time Time stamp
     * @param x Robot pose in x, y, z (NWU)
     * @param output_path Output destination for observed features
     * @param observed Observed features in both image and world frame
     */
    void recordObservedFeatures(double time,
                                const Vec3 &x,
                                const std::string &output_path,
                                std::vector<std::pair<Vec2, Vec3>> &observed);


    /** This class simulates a two wheel robot traversing in circle where the
     * world has 3D random feature points, and records:
     *
     * - Robot pose (robot ground truth)
     * - 3D Features in the world (landmark ground truth)
     * - 3D Features observed in image frame on the two wheel robot
     *
     * in separate files and in a dedicated folder.
     *
     * @param output_path Dataset destination
     */
    void generateTestData(const std::string &output_path);
};

/** @} end of group */
}  // namespace wave
#endif
