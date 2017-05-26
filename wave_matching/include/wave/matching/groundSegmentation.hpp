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
//#include "HLBFGS/HLBFGS.h"

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
    // variables
    double RMAX;       // max radius of point to consider
    int MAXBINPOINTS;  // max number of points to consider per bin
    int NUMSEEDPOINTS;

    // for GP model
    double P_L;   // length parameter, so how close the points have to be in the
                  // GP model to consider them correlated
    double P_SF;  // scaling on the whole covariance function
    double P_SN;  // the expected noise for the mode
    double P_TMODEL;  // the required confidence required in order to consider
                      // something ground
    double P_TDATA;   // scaled value that is required for a query point to be
                      // considered ground
    double P_TG;      // ground height threshold

    double ROBOT_HEIGHT;

    // seeding parameters
    double MAXSEEDRANGE;   // meters
    double MAXSEEDHEIGHT;  // meters

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

    // setters
    void set_rmax(double a) {
        RMAX = a;
    }
    void set_num_maxbinpoints(int a) {
        MAXBINPOINTS = a;
    }
    void set_num_seedpoints(int a) {
        NUMSEEDPOINTS = a;
    }

    void set_gp_lengthparameter(double a) {
        P_L = a;
    }
    void set_gp_covariancescale(double a) {
        P_SF = a;
    }
    void set_gp_modelnoise(double a) {
        P_SN = a;
    }
    void set_gp_groundmodelconfidence(double a) {
        P_TMODEL = a;
    }
    void set_gp_grounddataconfidence(double a) {
        P_TDATA = a;
    }
    void set_gp_groundthreshold(double a) {
        P_TG = a;
    }

    void set_robotheight(double a) {
        ROBOT_HEIGHT = a;
    }

    void set_seeding_maxrange(double a) {
        MAXSEEDRANGE = a;
    }
    void set_seeding_maxheight(double a) {
        MAXSEEDHEIGHT = a;
    }

 private:
    const int num_bins_a = 72;
    const int num_bins_l = 200;
};

#endif  // WAVE_GROUNDSEGMENTATION_HPP
