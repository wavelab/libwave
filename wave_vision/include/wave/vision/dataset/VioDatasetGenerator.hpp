/**
 * @file
 * @ingroup vision
 */
#ifndef WAVE_VISION_VIODATASETGENERATOR_HPP
#define WAVE_VISION_VIODATASETGENERATOR_HPP

#include "wave/vision/dataset/VioDataset.hpp"
#include "wave/vision/dataset/VoDataset.hpp"
#include "wave/kinematics/two_wheel.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */
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

    // Re-use base class `configure`
    using VoDatasetGenerator::configure;

    /** Simulates a two wheel robot moving in circle in a world of randomly
     * generated 3D point landmarks, and generates a VioDataset object.
     *
     * A measurement including robot pose is stored at every timestep (with an
     * arbitrary dt), but feature observations are only made at some timesteps.
     */
    VioDataset generate();
};

/** @} end of group */
}  // namespace wave
#endif  // WAVE_VISION_VIODATASETGENERATOR_HPP
