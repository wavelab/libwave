#ifndef WAVE_POINTTYPE_HPP
#define WAVE_POINTTYPE_HPP

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

/*
 * Point with extra information
 * intensity for gradient features on a plane (hopefully)
 * ring for easier id
 * ticks for figuring out the scan time based on scan period
 */
struct PointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;


// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                     (float, x, x)
                                     (float, y, y)
                                     (float, z, z)
                                     (float, intensity, intensity)
                                     (uint16_t, ring, ring))

// clang-format on

#endif  // WAVE_POINTTYPE_HPP
