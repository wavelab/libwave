/**
 * @file
 * Implementation of the full localizer state formulation which adhreres to REP
 * 105.
 * @ingroup gtsam
 */

#ifndef WAVE_LOCALIZER_STATE_HPP
#define WAVE_LOCALIZER_STATE_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

/**
 * This implements the traits required to optimize with gtsam
 * This state consists of:
 * T_odom_baselink
 * T_map_odom
 * T_ecef_map
 * vel_odom_baselink
 *
 * Follows implementation example of
 * https://github.com/dongjing3309/gtsam-examples/blob/master/cpp/Point2c.h
 * but is a bit different because all the members have traits already defined
 */

namespace localizer_state {
using VelType = Eigen::Matrix<double, 6, 1>;
}  // namespace localizer_state

namespace wave {

struct LocalizerState {
    /// Pose of the baselink frame with respect to the odom frame.
    gtsam::Pose3 T_odom_baselink;

    /// Pose of the odom frame with respect to the map frame.
    gtsam::Pose3 T_map_odom;

    /// Pose of the map frame with respect to the earch (ECEF) frame.
    gtsam::Pose3 T_ecef_map;

    /** Velocity of the baselink frame with respect to the odometry frame.
     * Expressed with angular values first, then linear.
     */
    localizer_state::VelType vel_odom_baselink;

    LocalizerState() {
        vel_odom_baselink.setZero();
    }

    enum {
        offset_odom_baselink = 0,
        offset_map_odom = 6,
        offset_ecef_map = 12,
        offset_vel_odom_baselink = 18
    };
};
}  // namespace wave

// All traits must be in the gtsam namespace.
namespace gtsam {
template <>

struct traits<wave::LocalizerState> {
    typedef lie_group_tag structure_category;

    /**
     * Basic (Testable)
     */

    static void Print(const wave::LocalizerState &m1,
                      const std::string &str = "") {
        traits<Pose3>::Print(m1.T_odom_baselink, str);
        traits<Pose3>::Print(m1.T_map_odom, str);
        traits<Pose3>::Print(m1.T_ecef_map, str);
        traits<localizer_state::VelType>::Print(m1.vel_odom_baselink, str);
    }

    static bool Equals(const wave::LocalizerState &m1,
                       const wave::LocalizerState &m2,
                       double tol = 1e-8) {
        if (!traits<Pose3>::Equals(
              m1.T_odom_baselink, m2.T_odom_baselink, tol)) {
            return false;
        }

        if (!traits<Pose3>::Equals(m1.T_map_odom, m2.T_map_odom, tol)) {
            return false;
        }

        if (!traits<Pose3>::Equals(m1.T_ecef_map, m2.T_ecef_map, tol)) {
            return false;
        }

        if (!traits<localizer_state::VelType>::Equals(
              m1.vel_odom_baselink, m2.vel_odom_baselink, tol)) {
            return false;
        }

        return true;
    }

    /**
     * Manifold
     */

    enum { dimension = 21 };
    static int GetDimension(const wave::LocalizerState &) {
        return dimension;
    }

    typedef wave::LocalizerState ManifoldType;
    typedef Eigen::Matrix<double, dimension, 1> TangentVector;

    // The tangent vector is just stacking all the smaller tangent vectors
    static TangentVector Local(const wave::LocalizerState &origin,
                               const wave::LocalizerState &other) {
        TangentVector retval;

        retval.block<6, 1>(0, 0).noalias() =
          traits<Pose3>::Local(origin.T_odom_baselink, other.T_odom_baselink);
        retval.block<6, 1>(6, 0).noalias() =
          traits<Pose3>::Local(origin.T_map_odom, other.T_map_odom);
        retval.block<6, 1>(12, 0).noalias() =
          traits<Pose3>::Local(origin.T_ecef_map, other.T_ecef_map);
        retval.block<6, 1>(18, 0).noalias() =
          traits<localizer_state::VelType>::Local(origin.vel_odom_baselink,
                                                  other.vel_odom_baselink);

        return retval;
    }

    static wave::LocalizerState Retract(const wave::LocalizerState &origin,
                                        const TangentVector &v) {
        wave::LocalizerState retval;

        retval.T_odom_baselink =
          traits<Pose3>::Retract(origin.T_odom_baselink, v.block<6, 1>(0, 0));
        retval.T_map_odom =
          traits<Pose3>::Retract(origin.T_map_odom, v.block<6, 1>(6, 0));
        retval.T_ecef_map =
          traits<Pose3>::Retract(origin.T_ecef_map, v.block<6, 1>(12, 0));
        retval.vel_odom_baselink = traits<localizer_state::VelType>::Retract(
          origin.vel_odom_baselink, v.block<6, 1>(18, 0));

        return retval;
    }

    /**
     * Lie group
     */

    typedef multiplicative_group_tag group_flavor;

    typedef OptionalJacobian<dimension, dimension> ChartJacobian;

    static wave::LocalizerState Identity() {
        return wave::LocalizerState();
    }

    static TangentVector Logmap(const wave::LocalizerState &m,
                                ChartJacobian Hm = boost::none) {
        TangentVector retval;

        Eigen::MatrixXd J1, J2, J3, J4;

        if (Hm) {
            Hm->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
            J3.resize(6, 6);
            J4.resize(6, 6);
        }

        retval.block<6, 1>(0, 0).noalias() =
          traits<Pose3>::Logmap(m.T_odom_baselink, J1);
        retval.block<6, 1>(6, 0).noalias() =
          traits<Pose3>::Logmap(m.T_map_odom, J2);
        retval.block<6, 1>(12, 0).noalias() =
          traits<Pose3>::Logmap(m.T_ecef_map, J3);
        retval.block<6, 1>(18, 0).noalias() =
          traits<localizer_state::VelType>::Logmap(m.vel_odom_baselink, J4);

        if (Hm) {
            Hm->block<6, 6>(0, 0).noalias() = J1;
            Hm->block<6, 6>(6, 6).noalias() = J2;
            Hm->block<6, 6>(12, 12).noalias() = J3;
            Hm->block<6, 6>(18, 18).noalias() = J4;
        }

        return retval;
    }

    static wave::LocalizerState Expmap(const TangentVector &v,
                                       ChartJacobian Hv = boost::none) {
        wave::LocalizerState retval;

        Eigen::MatrixXd J1, J2, J3, J4;

        if (Hv) {
            Hv->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
            J3.resize(6, 6);
            J4.resize(6, 6);
        }

        retval.T_odom_baselink = traits<Pose3>::Expmap(v.block<6, 1>(0, 0), J1);
        retval.T_map_odom = traits<Pose3>::Expmap(v.block<6, 1>(6, 0), J2);
        retval.T_ecef_map = traits<Pose3>::Expmap(v.block<6, 1>(12, 0), J3);
        retval.vel_odom_baselink =
          traits<localizer_state::VelType>::Expmap(v.block<6, 1>(18, 0), J4);

        if (Hv) {
            Hv->block<6, 6>(0, 0).noalias() = J1;
            Hv->block<6, 6>(6, 6).noalias() = J2;
            Hv->block<6, 6>(12, 12).noalias() = J3;
            Hv->block<6, 6>(18, 18).noalias() = J4;
        }

        return retval;
    }

    static wave::LocalizerState Compose(const wave::LocalizerState &m1,
                                        const wave::LocalizerState &m2,
                                        ChartJacobian H1 = boost::none,
                                        ChartJacobian H2 = boost::none) {
        wave::LocalizerState retval;

        Eigen::MatrixXd J1, J2, J3, J4, J5, J6, J7, J8;

        if (H1) {
            H1->setZero();
            J1.resize(6, 6);
            J3.resize(6, 6);
            J5.resize(6, 6);
            J7.resize(6, 6);
        }

        if (H2) {
            H2->setZero();
            J2.resize(6, 6);
            J4.resize(6, 6);
            J6.resize(6, 6);
            J8.resize(6, 6);
        }

        retval.T_odom_baselink = traits<Pose3>::Compose(
          m1.T_odom_baselink, m2.T_odom_baselink, J1, J2);
        retval.T_map_odom =
          traits<Pose3>::Compose(m1.T_map_odom, m2.T_map_odom, J3, J4);
        retval.T_ecef_map =
          traits<Pose3>::Compose(m1.T_ecef_map, m2.T_ecef_map, J5, J6);
        retval.vel_odom_baselink = traits<localizer_state::VelType>::Compose(
          m1.vel_odom_baselink, m2.vel_odom_baselink, J7, J8);

        if (H1) {
            H1->block<6, 6>(0, 0).noalias() = J1;
            H1->block<6, 6>(6, 6).noalias() = J3;
            H1->block<6, 6>(12, 12).noalias() = J5;
            H1->block<6, 6>(18, 18).noalias() = J7;
        }

        if (H2) {
            H2->block<6, 6>(0, 0).noalias() = J2;
            H2->block<6, 6>(6, 6).noalias() = J4;
            H2->block<6, 6>(12, 12).noalias() = J6;
            H2->block<6, 6>(18, 18).noalias() = J8;
        }

        return retval;
    }

    static wave::LocalizerState Between(const wave::LocalizerState &m1,
                                        const wave::LocalizerState &m2,
                                        ChartJacobian H1 = boost::none,
                                        ChartJacobian H2 = boost::none) {
        wave::LocalizerState retval;
        Eigen::MatrixXd J1, J2, J3, J4, J5, J6, J7, J8;

        if (H1) {
            H1->setZero();
            J1.resize(6, 6);
            J3.resize(6, 6);
            J5.resize(6, 6);
            J7.resize(6, 6);
        }

        if (H2) {
            H2->setZero();
            J2.resize(6, 6);
            J4.resize(6, 6);
            J6.resize(6, 6);
            J8.resize(6, 6);
        }

        retval.T_odom_baselink = traits<Pose3>::Between(
          m1.T_odom_baselink, m2.T_odom_baselink, J1, J2);
        retval.T_map_odom =
          traits<Pose3>::Between(m1.T_map_odom, m2.T_map_odom, J3, J4);
        retval.T_ecef_map =
          traits<Pose3>::Between(m1.T_ecef_map, m2.T_ecef_map, J5, J6);
        retval.vel_odom_baselink = traits<localizer_state::VelType>::Between(
          m1.vel_odom_baselink, m2.vel_odom_baselink, J7, J8);

        if (H1) {
            H1->block<6, 6>(0, 0).noalias() = J1;
            H1->block<6, 6>(6, 6).noalias() = J3;
            H1->block<6, 6>(12, 12).noalias() = J5;
            H1->block<6, 6>(18, 18).noalias() = J7;
        }

        if (H2) {
            H2->block<6, 6>(0, 0).noalias() = J2;
            H2->block<6, 6>(6, 6).noalias() = J4;
            H2->block<6, 6>(12, 12).noalias() = J6;
            H2->block<6, 6>(18, 18).noalias() = J8;
        }

        return retval;
    }

    static wave::LocalizerState Inverse(const wave::LocalizerState &m,
                                        ChartJacobian H = boost::none) {
        wave::LocalizerState retval;
        Eigen::MatrixXd J1, J2, J3, J4;

        if (H) {
            H->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
            J3.resize(6, 6);
            J4.resize(6, 6);
        }

        retval.T_odom_baselink = traits<Pose3>::Inverse(m.T_odom_baselink, J1);
        retval.T_map_odom = traits<Pose3>::Inverse(m.T_map_odom, J2);
        retval.T_ecef_map = traits<Pose3>::Inverse(m.T_ecef_map, J3);
        retval.vel_odom_baselink =
          traits<localizer_state::VelType>::Inverse(m.vel_odom_baselink, J4);

        if (H) {
            H->block<6, 6>(0, 0).noalias() = J1;
            H->block<6, 6>(6, 6).noalias() = J2;
            H->block<6, 6>(12, 12).noalias() = J3;
            H->block<6, 6>(18, 18).noalias() = J4;
        }

        return retval;
    }
};
}  // namespace gtsam

#endif  // WAVE_LOCALIZER_STATE_HPP
