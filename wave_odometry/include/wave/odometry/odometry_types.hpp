#ifndef WAVE_ODOMETRY_TYPES_HPP
#define WAVE_ODOMETRY_TYPES_HPP

#include <chrono>
#include <Eigen/Core>
#include "wave/geometry_og/transformation.hpp"
#include "wave/utils/math.hpp"

namespace wave {

template <int size>
using Earr = Eigen::array<long int, size>;
using ar1 = Earr<1>;
using ar2 = Earr<2>;
using ar3 = Earr<3>;

enum AssociationStatus { CORRESPONDED, UNCORRESPONDED };
enum ResidualType { PointToLine, PointToPlane };
enum SelectionPolicy { HIGH_POS, HIGH_NEG, NEAR_ZERO };
enum Kernel { LOAM, LOG, FOG, VAR };
enum Signal { RANGE, INTENSITY };

// true uses transform class with some approximations for exp map, etc
// false uses full analytical expressions wherever possible
using T_TYPE = Transformation<Eigen::Matrix<double, 3, 4>, true>;

using TimeType = std::chrono::steady_clock::time_point;

struct PoseVel {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseVel() {
        vel.setZero();
    }
    T_TYPE pose;
    Vec6 vel;
};

struct PoseVelStamped {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    T_TYPE pose;
    Vec6 vel;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> covar;
    TimeType stamp;
};

struct PoseStamped {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    T_TYPE pose;
    TimeType stamp;
};

struct PoseVelDiff {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec12 hat_multiplier, candle_multiplier;
};

struct Criteria {
    Signal signal;
    Kernel kernel;
    SelectionPolicy sel_pol;
    float *threshold;
};

struct FeatureDefinition {
    // First item defines sort, rest are logical.
    std::vector<Criteria> criteria;
    int *n_limit;
};


}

#endif  // WAVE_ODOMETRY_TYPES_HPP
