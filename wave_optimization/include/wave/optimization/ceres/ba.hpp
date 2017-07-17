#ifndef WAVE_OPTIMIZATION_CERES_BA_HPP
#define WAVE_OPTIMIZATION_CERES_BA_HPP

#include <typeinfo>

#include <ceres/ceres.h>

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"

namespace wave {

/** Bundle Adjustment Residual
 */
struct BAResidual {
 public:
    double fx;
    double fy;
    double cx;
    double cy;

    double x;
    double y;

    BAResidual() : fx(0.0), fy(0.0), cx(0.0), cy(0.0), x(0.0), y(0.0) {}

    BAResidual(Mat3 K, Vec2 x)
        : fx(K(0, 0)),
          fy(K(1, 1)),
          cx(K(0, 2)),
          cy(K(1, 2)),
          x(x(0)),
          y(x(1)) {}

    /**
       * Calculate Bundle Adjustment Residual
       *
     * @param q_GC Camera rotation parameterized with a quaternion (w, x, y, z)
     * @param G_p_C_G Camera translation (x, y, z)
     * @param world_pt World point (x, y, z)
     * @param residual Calculated residual
     **/
    template <typename T>
    bool operator()(const T *const q_GC,
                    const T *const G_p_C_G,
                    const T *const world_pt,
                    T *residual) const {
        // build camera intrinsics matrix K
        Eigen::Matrix<T, 3, 3> K;
        K(0, 0) = T(this->fx);
        K(0, 1) = T(0.0);
        K(0, 2) = T(this->cx);

        K(1, 0) = T(0.0);
        K(1, 1) = T(this->fy);
        K(1, 2) = T(this->cy);

        K(2, 0) = T(0.0);
        K(2, 1) = T(0.0);
        K(2, 2) = T(1.0);

        // build rotation matrix from quaternion q = (w, x, y, z)
        Eigen::Quaternion<T> q{q_GC[0], q_GC[1], q_GC[2], q_GC[3]};
        Eigen::Matrix<T, 3, 3> R_GC = q.toRotationMatrix();

        // build translation vector
        Eigen::Matrix<T, 3, 1> C{G_p_C_G[0], G_p_C_G[1], G_p_C_G[2]};

        // build world_point vector
        Eigen::Matrix<T, 3, 1> X{world_pt[0], world_pt[1], world_pt[2]};

        // get measurement
        Eigen::Matrix<T, 2, 1> est_pixel;
        pinholeProject(K, R_GC, C, X, est_pixel);

        // calculate residual error
        // diff between measured and estimated projection point
        residual[0] = ceres::abs(T(this->x) - est_pixel(0));
        residual[1] = ceres::abs(T(this->y) - est_pixel(1));

        return true;
    }
};

class BundleAdjustment {
 public:
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    BundleAdjustment() {}

    int addCamera(const Mat3 &K,
                  const MatX &features,
                  const VecX &landmark_ids,
                  double *cam_t,
                  double *cam_q,
                  double **landmarks);
    int solve();
};

}  // namespace wave
#endif
