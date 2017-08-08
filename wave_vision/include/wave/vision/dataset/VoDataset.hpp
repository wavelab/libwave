/**
 * @file
 * VO Synthetic Dataset Generator
 * class.
 * @ingroup vision
 */
#ifndef WAVE_VISION_VODATASET_HPP
#define WAVE_VISION_VODATASET_HPP

#include "wave/utils/utils.hpp"
#include "wave/vision/dataset/VoTestCamera.hpp"
#include "wave/kinematics/two_wheel.hpp"
#include "wave/containers/landmark_measurement.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** A set of feature observations at one timestep */
struct VoInstant {
    /** A time in nominal seconds */
    double time;

    /** The corresponding camera frame, or -1 if no camera observations */
    int camera_frame = -1;

    /** True robot Body position in the Global frame */
    Vec3 robot_G_p_GB;

    /** True robot Body orientation in the Global frame*/
    Quaternion robot_q_GB;

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
struct VoDataset {
    /** Ground truth 3D landmark positions in the world frame. */
    LandmarkMap landmarks;

    /** For each time step, a set of measurements */
    std::vector<VoInstant> states;

    /** Pinhole camera intrinsic calibration matrix */
    Mat3 camera_K;

    /**
     * Writes ground truth landmarks, states, and observations to the given
     * directory. If no errors occur, this is equivalent to calling
     *
     * ```
     * outputLandmarks(output_dir + "/landmarks.dat");
     * outputCalibration(output_dir + "/calib.dat");
     * outputRobotState(output_dir + "/state.dat");
     * outputObserved(output_dir);
     * ```
     *
     * @throws std::runtime_error on failure
     */
    void outputToDirectory(const std::string &output_dir);

    /** Writes landmark ground truth to the given file.
     *
     * The output is space-separated with no header. Each row contains the
     * landmark id and coordinates in x, y, z.
     *
     * @throws std::runtime_error on failure
     */
    void outputLandmarks(const std::string &output_path);

    /** Writes calibration information to the given file.
     *
     * Currently this is only camera calibration - this may change in the
     * future.
     *
     * The output is space-separated with only one row, which contains the nine
     * elements of the K matrix in row-major order.
     *
     * @throws std::runtime_error on failure
     */
    void outputCalibration(const std::string &output_path);

    /** Writes landmarks measurements to the given directory.
     *
     * For each timestep in the dataset, a file `observed_n.dat` is created
     * in output_dir, where n is an index starting at zero. Additionally, a file
     * index.dat is created holding a list of the `observed_*.dat` filenames.
     *
     * `observed_*.dat` is space-separated. There are four header rows:
     * 1. time
     * 2. robot position (x, y, z)
     * 3. robot orientation (quaternion x, y, z, w)
     * 4. M, the number of observations
     *
     * The next M rows each hold a landmark id and pixel measurements in the
     * image plane (u, v).
     *
     * @throws std::runtime_error on failure
     */
    void outputObserved(const std::string &output_dir);

    /** Writes robot state ground truth to the given file.
     *
     * The output is space-separated. Each row contains 8 values: the time, the
     * 3d position (x, y, z), and the quaternion orientation (x, y, z, w).
     *
     * @throws std::runtime_error on failure
     */
    void outputRobotState(const std::string &output_path);

    /** Reads a dataset from files in the given directory.
     *
     * The format must be the same as that created by outputToDirectory().
     *
     * @note: state.dat is not used as the information is stored redundantly in
     * the observed_* files. This may change (@todo).
     *
     * @todo: add error checking
     */
    static VoDataset loadFromDirectory(const std::string &input_dir);
};

/**
 * Synthetic VO dataset generator.
 *
 * This class simulates a two wheel robot traversing in circle where the world
 * has 3D random feature points, and stores the data in a VoTestDataset object.
 */
class VoDatasetGenerator {
 public:
    void configure(const std::string &config_file);

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
 * in a VoTestDataset object.
 *
 * A measurement including robot pose is stored at every timestep (with an
 * arbitrary dt), but feature observations are only made at some timesteps.
 */
    VoDataset generate();

    VoTestCamera camera;
    int nb_landmarks = 0;
    Vec2 landmark_x_bounds = Vec2::Zero();
    Vec2 landmark_y_bounds = Vec2::Zero();
    Vec2 landmark_z_bounds = Vec2::Zero();
};

/** @} end of group */
}  // namespace wave
#endif  // WAVE_VISION_VODATASET_HPP
