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
    uint8_t intensity;
    uint8_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR, (float, x, x)
        (float, y, y)
        (float, z, z)
        (uint8_t, intensity, intensity)
        (uint8_t, ring, ring))

#endif //WAVE_POINTTYPE_HPP
