
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

#ifndef WAVE_POINTCLOUDXYZGD_HPP
#define WAVE_POINTCLOUDXYZGD_HPP

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZGD {
    PCL_ADD_POINT4D;                 // quad-word XYZ
    uint16_t ground_adj;             ///< ground adjacent flag
    uint16_t drivable;               ///< driveable flag
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZGD,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                    uint16_t, ground_adj, ground_adj)(uint16_t,
                                                                      drivable,
                                                                      drivable))

// Helper functions
inline bool isDR(float in) {
    return (in > 0.7);
}

inline bool isGA(float in) {
    return (in > 0.5);
}

#endif  // WAVE_POINTCLOUDXYZGD_HPP
