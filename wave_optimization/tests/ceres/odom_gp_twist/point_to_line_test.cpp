#include <ceres/ceres.h>
#include <wave/wave_test.hpp>
#include "wave/optimization/ceres/odom_gp_twist/point_to_line_gp_full.hpp"

namespace wave_optimization {

TEST(point_to_line, initialization) {
    SE3PointToLineGPFullObject object;
}

}