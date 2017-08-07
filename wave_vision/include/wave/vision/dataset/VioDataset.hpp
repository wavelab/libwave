/**
 * @file
 * @ingroup vision
 */
#ifndef WAVE_VISION_VIODATASET_HPP
#define WAVE_VISION_VIODATASET_HPP

#include "wave/utils/utils.hpp"
#include "wave/vision/dataset/VoDataset.hpp"
#include "wave/containers/measurement_container.hpp"
#include "wave/containers/landmark_measurement_container.hpp"
#include "wave/containers/measurement.hpp"
#include "wave/containers/landmark_measurement.hpp"
#include "wave/kinematics/two_wheel.hpp"
#include "wave/geometry/rotation.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/**
 * A set of pre-recorded visual and inertial measurements.
 * The data could be synthetic or real.
 *
 * This includes:
 * - GPS position (can be used as ground truth)
 * - 3D landmark positions in the world (if available, as ground truth)
 * - Landmark measurements in the image frame
 * - Inertial measurements in the IMU frame
 * - Intrinsic and extrinsic calibration
 *
 * @todo In this version, inertial measurements include velocity, not
 * acceleration. This may change.
 * @todo Currently only one camera is included.
 */
struct VioDataset {
    // Types
    // @todo - unify with other definitions

    /** IMU measured value - note this version includes velocity.
     * The affixes indicate it is the motion of the IMU with respect to the
     * the world frame, expressed in the world frame */
    struct ImuValue {
        Vec3 G_vel_GI;
        Vec3 G_ang_vel_GI;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /** Pose value - either synthetic ground truth or from GPS and pre-processed
     *  IMU data. The affixes indicate it is the motion of the IMU with respect
     *  to the world frame, expressed in the world frame */
    struct PoseValue {
        Vec3 G_p_GI;
        Rotation R_GI;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    enum class ImuSensor { Imu };
    enum class PoseSensor { Gps };
    enum class Camera { Left };
    using PoseMeasurement = Measurement<Vec3, PoseSensor>;
    using ImuMeasurement = Measurement<Vec3, ImuSensor>;

    // Measurements
    MeasurementContainer<PoseMeasurement> poses;
    MeasurementContainer<ImuMeasurement> imu_measurements;
    LandmarkMeasurementContainer<LandmarkMeasurement<Camera>>
      feature_measurements;

    // Calibration

    /** Camera intrinsic matrix */
    Mat3 calib_K;
    /** Camera transformation from IMU frame (expressed in IMU frame) */
    Vec3 I_p_IC;
    Rotation R_IC;

    /** Ground truth 3D world landmarks, where each column represents a feature
     * and each row represents the feature position in x, y, z (NWU)  */
    EigenVector<Vec3> landmarks;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Synthetic VIO dataset generator.
 *
 * Currently uses the interface and parameters of VoDatasetGenerator,
 * just defining another generate() method.
 */
class VioDatasetGenerator : private VoDatasetGenerator {
 public:
    // Inherit base class constructors
    using VoDatasetGenerator::VoDatasetGenerator;

    /** Simulates a two wheel robot moving in circle in a world of randomly
     * generated 3D point landmarks, and generates a VioDataset object.
     *
     * A measurement including robot pose is stored at every timestep (with an
     * arbitrary dt), but feature observations are only made at some timesteps.
     */
    VioDataset generate() const;
};


/** @} end of group */
}  // namespace wave
#endif  // WAVE_VISION_VIODATASET_HPP
