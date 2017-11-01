/** @file
 * @ingroup matching
 * @author Arun Das (adas@uwaterloo.ca)
 * @author James Servos (jdservos@uwaterloo.ca)
 *
 * Gaussian Process Ground Segmentation
 *
 * Based on approach in
 * https://link.springer.com/article/10.1007/s10846-013-9889-4
 *
 */

#ifndef WAVE_GROUNDSEGMENTATION_HPP
#define WAVE_GROUNDSEGMENTATION_HPP

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <wave/matching/PointcloudXYZGD.hpp>
#include <wave/matching/ground_segmentation_params.hpp>
#include <wave/utils/math.hpp>

namespace wave {
/** @addtogroup matching
*  @{ */

/**
 *  Structure to hold point data for ground model
 */
struct SignalPoint {
    double range;
    double height;
    int index;
    bool is_ground;
};

/**
 * Structure to hold collection of points in each linear bin
 */
struct LinCell {
    std::vector<PointXYZGD> bin_points;  // all the points
    std::vector<PointXYZGD> obs_points;  // just the obs points
    std::vector<PointXYZGD> drv_points;
    std::vector<PointXYZGD> ground_points;  // just the ground points
    PointXYZGD prototype_point;
    int cluster_assigned;  // what cluster is it assigned to
    Vec3 obs_mean;         // mean of obstacle points
};

/**
 *  Structure to hold all linear bins in that angular bin
 */
struct AngCell {
    std::vector<SignalPoint> sig_points;  // range height signal for that sector
    std::vector<LinCell> lin_cell;  // the linear cells within that angle sector
    std::vector<pcl::PointXY> range_height_signal;
};

/**
*  Structure to hold all angular bins
*/
struct PolarBinGrid {
    std::vector<AngCell> ang_cell;
};

/**
 * This is a class to perform ground segmentation on pointclouds.
 * Can be a useful preprocessing step.
 */
class GroundSegmentation {
 public:
    /**
     * Collection of all configurable parameters
     */
    GroundSegmentationParams params;

    /**
     * Data storage
     */
    PolarBinGrid *polar_bin_grid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud;
    pcl::PointCloud<PointXYZGD>::Ptr ground_cloud;  // ground points
    pcl::PointCloud<PointXYZGD>::Ptr obs_cloud;     // obstacle points
    pcl::PointCloud<PointXYZGD>::Ptr drv_cloud;     // drivable points

    /**
     * Constructor
     * @param config: GroundSegmentationParams settings
     */
    GroundSegmentation(GroundSegmentationParams config);

    /**
     * Used to give pointcloud to segmentor as well as pointers to
     * return segmented clouds
     *
     * @param[in]input_cloud - Pointer of input cloud
     * @param[out]ground_cloud - pointer of cloud to place all ground points
     * @param[out]obs_cloud - pointer of cloud to place all obstacle points
     * @param[out]drv_cloud - pointer of cloud to place all drivable points
     */
    void setupGroundSegmentation(
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
      pcl::PointCloud<PointXYZGD>::Ptr out_ground_cloud,
      pcl::PointCloud<PointXYZGD>::Ptr out_obs_cloud,
      pcl::PointCloud<PointXYZGD>::Ptr out_drv_cloud);

    /**
     * Bins all points from the input cloud
     *
     * @param[in] input_cloud
     */
    void genPolarBinGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

    /**
     * Resets bin data structure
     */
    void initializePolarBinGrid(void);

    /**
     * Calculates Gram Matrix for given data points. Squared exponential kernel
     *
     * @param[in] ps1 First set of signal points
     * @param[in] ps2 Second set of signal points
     * @param[in] p_l Kernel hyperparameter
     * @param[in] sig_f Kernel hyperparameter
     * @return  Gram Matrix
     */
    MatX genGPModel(std::vector<SignalPoint> &ps1,
                    std::vector<SignalPoint> &ps2,
                    float sig_f,
                    float p_l);

    /**
     * Segments ground
     */
    void segmentGround(void);

    /**
     * Iteratively builds up ground model
     */
    void sectorINSAC(int);
};

}  // namespace wave

#endif  // WAVE_GROUNDSEGMENTATION_HPP
