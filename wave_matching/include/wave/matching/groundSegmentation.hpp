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

#include <ros/console.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <geometry_msgs/Pose.h>
#include "ground_segmentation/PointcloudXYZGD.h"
#include "groundSegmentationParams.hpp"

// Defines
#define INVALID 1000

struct signalPoint {
    double range;
    double height;
    int idx;
    bool isGround;
};

struct linCell {
    vector<PointXYZGD> binPoints;  // all the points
    vector<PointXYZGD> obsPoints;  // just the obs points
    vector<PointXYZGD> drvPoints;
    vector<PointXYZGD> groundPoints;  // just the ground points
    PointXYZGD prototypePoint;
    int cAssigned;            // what cluster is it assigned to
    Eigen::Vector3d obsMean;  // mean of obstacle points
};

struct angCell {
    linCell lCell[NUMBINSL];  // the linear cells within that angle sector
    pcl::PointXY rangeHeightSignal[NUMBINSL];
    vector<signalPoint> sigPoints;  // range height signal for that sector
};

struct polarBinGrid {
    angCell aCell[NUMBINSA];
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
    Eigen::MatrixXd genGPModel(vector<signalPoint> &,
                               vector<signalPoint> &,
                               float,
                               float);
    void segmentGround(void);
    void sectorINSAC(int);
 private:
};

#endif  // WAVE_GROUNDSEGMENTATION_HPP
