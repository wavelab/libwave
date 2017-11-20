#ifndef WAVE_WEIGHTING_FUNCTIONS_HPP
#define WAVE_WEIGHTING_FUNCTIONS_HPP

#include <Eigen/Core>

namespace wave {

Eigen::Matrix3d calculatePointToLineWeight(const double *const pt,
                                           const double *const ptA,
                                           const double *const ptB,
                                           const double &variance);

double calculatePointToPlaneWeight(const double *const pt,
                                   const double *const ptA,
                                   const double *const ptB,
                                   const double *const ptC,
                                   const double &variance);

}

#endif //WAVE_WEIGHTING_FUNCTIONS_HPP
