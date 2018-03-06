#include "wave/gtsam/localizer_state.hpp"

namespace wave {

LocalizerState operator*(const LocalizerState &m1, const LocalizerState &m2) {
    LocalizerState retval;

    retval.T_odom_baselink = m1.T_odom_baselink * m2.T_odom_baselink;
    retval.T_map_odom = m1.T_map_odom * m2.T_map_odom;
    retval.T_ecef_map = m1.T_ecef_map * m2.T_ecef_map;
    retval.vel_odom_baselink = m1.vel_odom_baselink + m2.vel_odom_baselink;

    return retval;
}
}
