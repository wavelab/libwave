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

// Defines
namespace wave {

#define INVALID 1000

struct signalPoint {
    double range;
    double height;
    int idx;
    bool isGround;
};

struct linCell {
    std::vector<PointXYZGD> binPoints;  // all the points
    std::vector<PointXYZGD> obsPoints;  // just the obs points
    std::vector<PointXYZGD> drvPoints;
    std::vector<PointXYZGD> groundPoints;  // just the ground points
    PointXYZGD prototypePoint;
    int cAssigned;  // what cluster is it assigned to
    Vec3 obsMean;   // mean of obstacle points
};

struct angCell {
    std::vector<signalPoint> sigPoints;  // range height signal for that sector
    std::vector<linCell> lCell;  // the linear cells within that angle sector
    std::vector<pcl::PointXY> rangeHeightSignal;
};

struct polarBinGrid {
    std::vector<angCell> aCell;
};

class groundSegmentation {
 public:
    groundSegmentationParams params;
    // data structures
    polarBinGrid *pBG;
    int test;
    pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud;
    pcl::PointCloud<PointXYZGD>::Ptr gCloud;  // ground points
    pcl::PointCloud<PointXYZGD>::Ptr oCloud;  // obstacle points
    pcl::PointCloud<PointXYZGD>::Ptr dCloud;  // obstacle points

    // constructor
    groundSegmentation();

    void setupGroundSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr,
                                 pcl::PointCloud<PointXYZGD>::Ptr,
                                 pcl::PointCloud<PointXYZGD>::Ptr,
                                 pcl::PointCloud<PointXYZGD>::Ptr);

    void genPolarBinGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr);

    void initializePolarBinGrid(void);

    MatX genGPModel(std::vector<signalPoint> &,
                    std::vector<signalPoint> &,
                    float,
                    float);

    void segmentGround(void);

    void sectorINSAC(int);

 private:
};

}  // namespace wave

#endif  // WAVE_GROUNDSEGMENTATION_HPP
