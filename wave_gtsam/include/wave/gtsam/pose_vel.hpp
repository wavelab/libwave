#ifndef WAVE_POSE_VEL_HPP
#define WAVE_POSE_VEL_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

/**
 * This implements the traits required to optimize with gtsam
 * This state is combined pose and velocity
 * Follows implementation example of
 * https://github.com/dongjing3309/gtsam-examples/blob/master/cpp/Point2c.h
 * but is a bit different because all the members have traits already defined
 */

namespace {
using VelType = Eigen::Matrix<double, 6, 1>;
}

namespace wave {

// todo: Figure out how to template dimension of bias
struct PoseVel {
    gtsam::Pose3 pose;

    /// Angular followed by linear
    VelType vel;

    PoseVel() {
        vel.setZero();
    }
};
}

namespace gtsam {

template <>
struct traits<wave::PoseVel> {
    typedef lie_group_tag structure_category;

    /**
     * Basic (Testable)
     */

    static void Print(const wave::PoseVel &m1, const std::string &str = "") {
        traits<Pose3>::Print(m1.pose, str);
        traits<VelType>::Print(m1.vel, str);
    }

    static bool Equals(const wave::PoseVel &m1,
                       const wave::PoseVel &m2,
                       double tol = 1e-8) {
        if (!traits<Pose3>::Equals(m1.pose, m2.pose, tol)) {
            return false;
        }
        if (!traits<VelType>::Equals(m1.vel, m2.vel, tol)) {
            return false;
        }
        return true;
    }

    /**
     * Manifold
     */

    enum { dimension = 12 };
    static int GetDimension(const wave::PoseVel &) {
        return dimension;
    }

    typedef wave::PoseVel ManifoldType;
    typedef Eigen::Matrix<double, dimension, 1> TangentVector;

    // The tangent vector is just stacking all the smaller tangent vectors
    static TangentVector Local(const wave::PoseVel &origin,
                               const wave::PoseVel &other) {
        TangentVector retval;
        retval.block<6, 1>(0, 0).noalias() =
          traits<Pose3>::Local(origin.pose, other.pose);
        retval.block<6, 1>(6, 0).noalias() =
          traits<VelType>::Local(origin.vel, other.vel);
        return retval;
    }

    static wave::PoseVel Retract(const wave::PoseVel &origin,
                                 const TangentVector &v) {
        wave::PoseVel retval;
        retval.pose = traits<Pose3>::Retract(origin.pose, v.block<6, 1>(0, 0));
        retval.vel = traits<VelType>::Retract(origin.vel, v.block<6, 1>(6, 0));
        return retval;
    }

    /**
     * Lie group
     */

    typedef multiplicative_group_tag group_flavor;

    typedef OptionalJacobian<dimension, dimension> ChartJacobian;

    static wave::PoseVel Identity() {
        return wave::PoseVel();
    }

    static TangentVector Logmap(const wave::PoseVel &m,
                                ChartJacobian Hm = boost::none) {
        Eigen::MatrixXd J1, J2;
        TangentVector retval;
        if (Hm) {
            Hm->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
        }
        retval.block<6, 1>(0, 0).noalias() = traits<Pose3>::Logmap(m.pose, J1);
        retval.block<6, 1>(6, 0).noalias() = traits<VelType>::Logmap(m.vel, J2);
        if (Hm) {
            Hm->block<6, 6>(0, 0).noalias() = J1;
            Hm->block<6, 6>(6, 6).noalias() = J2;
        }
        return retval;
    }

    static wave::PoseVel Expmap(const TangentVector &v,
                                ChartJacobian Hv = boost::none) {
        wave::PoseVel retval;
        Eigen::MatrixXd J1, J2;
        if (Hv) {
            Hv->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
        }
        retval.pose = traits<Pose3>::Expmap(v.block<6, 1>(0, 0), J1);
        retval.vel = traits<VelType>::Logmap(v.block<6, 1>(6, 0), J2);
        if (Hv) {
            Hv->block<6, 6>(0, 0).noalias() = J1;
            Hv->block<6, 6>(6, 6).noalias() = J2;
        }
        return retval;
    }

    static wave::PoseVel Compose(const wave::PoseVel &m1,
                                 const wave::PoseVel &m2,
                                 ChartJacobian H1 = boost::none,
                                 ChartJacobian H2 = boost::none) {
        wave::PoseVel retval;
        Eigen::MatrixXd J1, J2, J3, J4;
        if (H1) {
            H1->setZero();
            J1.resize(6, 6);
            J3.resize(6, 6);
        }
        if (H2) {
            H2->setZero();
            J2.resize(6, 6);
            J4.resize(6, 6);
        }
        retval.pose = traits<Pose3>::Compose(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<VelType>::Compose(m1.vel, m2.vel, J3, J4);
        if (H1) {
            H1->block<6, 6>(0, 0).noalias() = J1;
            H1->block<6, 6>(6, 6).noalias() = J3;
        }
        if (H2) {
            H2->block<6, 6>(0, 0).noalias() = J2;
            H2->block<6, 6>(6, 6).noalias() = J4;
        }
        return retval;
    }

    static wave::PoseVel Between(const wave::PoseVel &m1,
                                 const wave::PoseVel &m2,
                                 ChartJacobian H1 = boost::none,
                                 ChartJacobian H2 = boost::none) {
        wave::PoseVel retval;
        Eigen::MatrixXd J1, J2, J3, J4;
        if (H1) {
            H1->setZero();
            J1.resize(6, 6);
            J3.resize(6, 6);
        }
        if (H2) {
            H2->setZero();
            J2.resize(6, 6);
            J4.resize(6, 6);
        }
        retval.pose = traits<Pose3>::Between(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<VelType>::Between(m1.vel, m2.vel, J3, J4);
        if (H1) {
            H1->block<6, 6>(0, 0).noalias() = J1;
            H1->block<6, 6>(6, 6).noalias() = J3;
        }
        if (H2) {
            H2->block<6, 6>(0, 0).noalias() = J2;
            H2->block<6, 6>(6, 6).noalias() = J4;
        }
        return retval;
    }

    static wave::PoseVel Inverse(const wave::PoseVel &m,
                                 ChartJacobian H = boost::none) {
        wave::PoseVel retval;
        Eigen::MatrixXd J1, J2;
        if (H) {
            H->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
        }
        retval.pose = traits<Pose3>::Inverse(m.pose, J1);
        retval.vel = traits<VelType>::Inverse(m.vel, J2);
        if (H) {
            H->block<6, 6>(0, 0).noalias() = J1;
            H->block<6, 6>(6, 6).noalias() = J2;
        }
        return retval;
    }
};
}

namespace wave {

PoseVel operator*(const PoseVel &m1, const PoseVel &m2) {
    wave::PoseVel retval;
    retval.pose = m1.pose * m2.pose;
    retval.vel = m1.vel + m2.vel;
    return retval;
}
}

#endif  // WAVE_POSE_VEL_HPP
