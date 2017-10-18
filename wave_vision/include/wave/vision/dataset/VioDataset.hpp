/**
 * @file
 * @ingroup vision
 */
#ifndef WAVE_VISION_VIODATASET_HPP
#define WAVE_VISION_VIODATASET_HPP

#include "wave/utils/utils.hpp"
#include "wave/containers/measurement_container.hpp"
#include "wave/containers/landmark_measurement_container.hpp"
#include "wave/containers/measurement.hpp"
#include "wave/containers/landmark_measurement.hpp"
#include "wave/geometry/rotation.hpp"
#include "wave/vision/dataset/VoTestCamera.hpp"

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
     * the world frame, expressed in the IMU frame */
    struct ImuValue {
        Vec3 I_vel_GI;
        Vec3 I_ang_vel_GI;
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
    enum class PoseSensor { Pose };
    enum class Camera { Left };

    using ObsContainer =
      LandmarkMeasurementContainer<LandmarkMeasurement<Camera>>;
    using PoseContainer =
      MeasurementContainer<Measurement<PoseValue, PoseSensor>>;
    using ImuContainer = MeasurementContainer<Measurement<ImuValue, ImuSensor>>;

    // Measurements
    PoseContainer poses;
    ImuContainer imu_measurements;
    ObsContainer feature_measurements;

    // Calibration

    /** Camera intrinsic matrix */
    Mat3 camera_K;
    /** Camera transformation from IMU frame (expressed in IMU frame) */
    Vec3 I_p_IC;
    Rotation R_IC;

    /** Ground truth 3D landmark positions in the world frame. */
    LandmarkMap landmarks;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @} end of group */
}  // namespace wave
#endif  // WAVE_VISION_VIODATASET_HPP
