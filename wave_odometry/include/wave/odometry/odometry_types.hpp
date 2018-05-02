#ifndef WAVE_ODOMETRY_TYPES_HPP
#define WAVE_ODOMETRY_TYPES_HPP

#include <Eigen/Core>
#include "wave/geometry/transformation.hpp"
#include "wave/utils/math.hpp"

namespace wave {

enum AssociationStatus { CORRESPONDED, UNCORRESPONDED };
enum ResidualType { PointToLine, PointToPlane };
enum SelectionPolicy { HIGH_POS, HIGH_NEG, NEAR_ZERO };
enum Kernel { LOAM, LOG, FOG, RNG_VAR, INT_VAR };
enum Signal { RANGE, INTENSITY };

// true uses transform class with some approximations for exp map, etc
// false uses full analytical expressions wherever possible
using T_TYPE = Transformation<Eigen::Matrix<double, 3, 4>, true>;

struct Trajectory {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    T_TYPE pose;
    Vec6 vel;
};

struct TrajDifference {
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
