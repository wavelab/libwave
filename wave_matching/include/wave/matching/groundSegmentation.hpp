/** @file
 * @ingroup matching
 *
 * Gaussian Process Ground Segmentation
 *
 */

#ifndef WAVE_GROUNDSEGMENTATION_HPP
#define WAVE_GROUNDSEGMENTATION_HPP

/************************************************************************
 *
 *
 *  Copyright 2015  Arun Das (University of Waterloo)
 *                      [adas@uwaterloo.ca]
 *                  James Servos (University of Waterloo)
 *                      [jdservos@uwaterloo.ca]
 *
 *
 *************************************************************************/

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <wave/matching/PointcloudXYZGD.hpp>
#include <wave/matching/groundSegmentationParams.hpp>
#include <wave/utils/math.hpp>

namespace wave {
/** @addtogroup matching
*  @{ */

/**
 *  Structure to hold point data for ground model
 */
struct signalPoint {
    double range;
    double height;
    int idx;
    bool isGround;
};

/**
 * Structure to hold collection of points in each linear bin
 */
struct linCell {
    std::vector<PointXYZGD> binPoints;  // all the points
    std::vector<PointXYZGD> obsPoints;  // just the obs points
    std::vector<PointXYZGD> drvPoints;
    std::vector<PointXYZGD> groundPoints;  // just the ground points
    PointXYZGD prototypePoint;
    int cAssigned;  // what cluster is it assigned to
    Vec3 obsMean;   // mean of obstacle points
};

/**
 *  Structure to hold all linear bins in that angular bin
 */
struct angCell {
    std::vector<signalPoint> sigPoints;  // range height signal for that sector
    std::vector<linCell> lCell;  // the linear cells within that angle sector
    std::vector<pcl::PointXY> rangeHeightSignal;
};

/**
*  Structure to hold all angular bins
*/
struct polarBinGrid {
    std::vector<angCell> aCell;
};

/**
 * This is a class to perform ground segmentation on pointclouds.
 * Can be a useful preprocessing step.
 */
class groundSegmentation {
 public:
    /**
     * Collection of all configurable parameters
     */
    GroundSegmentationParams params;

    /**
     * Data storage
     */
    polarBinGrid *pBG;
    pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud;
    pcl::PointCloud<PointXYZGD>::Ptr gCloud;  // ground points
    pcl::PointCloud<PointXYZGD>::Ptr oCloud;  // obstacle points
    pcl::PointCloud<PointXYZGD>::Ptr dCloud;  // drivable points

    /**
     * Constructor
     * @param config: GroundSegmentationParams settings
     */
    groundSegmentation(GroundSegmentationParams config);

    /**
     * Used to give pointcloud to segmentor as well as pointers to
     * return segmented clouds
     *
     * @param[in]inputCloud - Pointer of input cloud
     * @param[out]drvCloud - pointer of cloud to place all drivable points
     * @param[out]groundCloud - pointer of cloud to place all ground points
     * @param[out]obsCloud - pointer of cloud to place all obstacle points
     */
    void setupGroundSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                                 pcl::PointCloud<PointXYZGD>::Ptr groundCloud,
                                 pcl::PointCloud<PointXYZGD>::Ptr obsCloud,
                                 pcl::PointCloud<PointXYZGD>::Ptr drvCloud);

    /**
     * Bins all points from the input cloud
     *
     * @param[in] inputCloud
     */
    void genPolarBinGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

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
    MatX genGPModel(std::vector<signalPoint> &ps1,
                    std::vector<signalPoint> &ps2,
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
