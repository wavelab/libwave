#ifndef __wave_OPTIMIZATION_OPTIMIZERS_RANSAC_HPP__
#define __wave_OPTIMIZATION_OPTIMIZERS_RANSAC_HPP__

#include <iostream>
#include <math.h>

#include <Eigen/Dense>

#include "wave/utils/math.hpp"
#include "wave/utils/utils.hpp"


namespace wave {

class RANSAC {
public:
  bool configured;

  int max_iter;
  double thresh_ratio;
  double thresh_dist;

  int iter;
  std::vector<int> inliers;
  double threshold;
  int max_inliers;
  double model_params[2];

  RANSAC(void);
  int configure(int max_iter, double threshold_ratio, double threshold_dist);
  int randomSample(MatX &data, Vec2 &sample);
  int computeDistances(MatX &data, Vec2 &p1, Vec2 &p2, VecX &dists);
  int computeInliers(VecX &dists);
  int update(Vec2 &p1, Vec2 &p2);
  int printStats(void);
  int optimize(MatX &data);
};

}  // end of wave namespace
#endif
