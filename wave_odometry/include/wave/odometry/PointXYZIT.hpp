#ifndef WAVE_POINTXYZIT_HPP
#define WAVE_POINTXYZIT_HPP

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

/*
 * Point with extra information
 * tick for determining time in scan
 */
struct PointXYZIT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t tick;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;


// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                     (float, x, x)
                                     (float, y, y)
                                     (float, z, z)
                                     (float, intensity, intensity)
                                     (uint16_t, tick, tick))

// clang-format on

#endif  // WAVE_POINTXYZIT_HPP
