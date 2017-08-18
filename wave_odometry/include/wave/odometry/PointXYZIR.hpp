#ifndef WAVE_POINTXYZIR_HPP
#define WAVE_POINTXYZIR_HPP

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

/*
 * Point with extra information
 * intensity for gradient features on a plane (hopefully)
 * ring for easier id
 */
namespace wave {

struct PointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

};  //namespace wave

// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(wave::PointXYZIR,
                                     (float, x, x)
                                     (float, y, y)
                                     (float, z, z)
                                     (float, intensity, intensity)
                                     (uint16_t, ring, ring))

// clang-format on

#endif  // WAVE_POINTXYZIR_HPP
