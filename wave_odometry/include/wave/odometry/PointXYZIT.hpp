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
struct PCLPointXYZIT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t tick;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;


// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(PCLPointXYZIT,
                                     (float, x, x)
                                     (float, y, y)
                                     (float, z, z)
                                     (float, intensity, intensity)
                                     (uint32_t, tick, tick))

// clang-format on

struct PointXYZIT {
    PointXYZIT(double x, double y, double z, float inten, uint32_t ticks)
        : pt{x, y, z}, intensity(inten), tick(ticks) {}
    double pt[3];
    float intensity;
    uint32_t tick;
};

#endif  // WAVE_POINTXYZIT_HPP
