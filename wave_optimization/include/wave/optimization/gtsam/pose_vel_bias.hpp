#ifndef WAVE_POSE_VEL_BIAS_HPP
#define WAVE_POSE_VEL_BIAS_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

/**
 * This implements the traits required to optimize with gtsam
 * This state is combined pose, velocity, and some of biases.
 * Follows implementation example of
 * https://github.com/dongjing3309/gtsam-examples/blob/master/cpp/Point2c.h
 * but is a bit different because all the members have traits already defined
 */

namespace {
using VelType = Eigen::Matrix<double, 6, 1>;
using BiasType = Eigen::Matrix<double, 3, 1>;
}

namespace wave {

// todo: Figure out how to template dimension of bias
struct PoseVelBias {
    gtsam::Pose3 pose;
    /// Angular followed by linear
    VelType vel;
    BiasType bias;

    PoseVelBias() {
        vel.setZero();
        bias.setZero();
    }

    enum { pose_offset = 0, vel_offset = 6, bias_offset = 12 };
};
}

namespace gtsam {

template <>
struct traits<wave::PoseVelBias> {
    typedef lie_group_tag structure_category;

    /**
     * Basic (Testable)
     */

    static void Print(const wave::PoseVelBias &m1,
                      const std::string &str = "") {
        traits<Pose3>::Print(m1.pose, str);
        traits<VelType>::Print(m1.vel, str);
        traits<BiasType>::Print(m1.bias, str);
    }

    static bool Equals(const wave::PoseVelBias &m1,
                       const wave::PoseVelBias &m2,
                       double tol = 1e-8) {
        if (!traits<Pose3>::Equals(m1.pose, m2.pose, tol)) {
            return false;
        }
        if (!traits<VelType>::Equals(m1.vel, m2.vel, tol)) {
            return false;
        }
        if (!traits<BiasType>::Equals(m1.bias, m2.bias, tol)) {
            return false;
        }
        return true;
    }

    /**
     * Manifold
     */

    enum { dimension = 15 };
    static int GetDimension(const wave::PoseVelBias &) {
        return dimension;
    }

    typedef wave::PoseVelBias ManifoldType;
    typedef Eigen::Matrix<double, dimension, 1> TangentVector;

    // The tangent vector is just stacking all the smaller tangent vectors
    static TangentVector Local(const wave::PoseVelBias &origin,
                               const wave::PoseVelBias &other) {
        TangentVector retval;
        retval.block<6, 1>(0, 0).noalias() =
          traits<Pose3>::Local(origin.pose, other.pose);
        retval.block<6, 1>(6, 0).noalias() =
          traits<VelType>::Local(origin.vel, other.vel);
        retval.block<3, 1>(12, 0).noalias() =
          traits<BiasType>::Local(origin.bias, other.bias);
        return retval;
    }

    static wave::PoseVelBias Retract(const wave::PoseVelBias &origin,
                                     const TangentVector &v) {
        wave::PoseVelBias retval;
        retval.pose = traits<Pose3>::Retract(origin.pose, v.block<6, 1>(0, 0));
        retval.vel = traits<VelType>::Retract(origin.vel, v.block<6, 1>(6, 0));
        retval.bias =
          traits<BiasType>::Retract(origin.bias, v.block<3, 1>(12, 0));
        return retval;
    }

    /**
     * Lie group
     */

    typedef multiplicative_group_tag group_flavor;

    typedef OptionalJacobian<dimension, dimension> ChartJacobian;

    static wave::PoseVelBias Identity() {
        return wave::PoseVelBias();
    }

    static TangentVector Logmap(const wave::PoseVelBias &m,
                                ChartJacobian Hm = boost::none) {
        Eigen::MatrixXd J1, J2, J3;

        TangentVector retval;
        if (Hm) {
            Hm->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
            J3.resize(3, 3);
        }
        retval.block<6, 1>(0, 0).noalias() = traits<Pose3>::Logmap(m.pose, J1);
        retval.block<6, 1>(6, 0).noalias() = traits<VelType>::Logmap(m.vel, J2);
        retval.block<3, 1>(12, 0).noalias() =
          traits<BiasType>::Logmap(m.bias, J3);
        if (Hm) {
            Hm->block<6, 6>(0, 0).noalias() = J1;
            Hm->block<6, 6>(6, 6).noalias() = J2;
            Hm->block<3, 3>(12, 12).noalias() = J3;
        }
        return retval;
    }

    static wave::PoseVelBias Expmap(const TangentVector &v,
                                    ChartJacobian Hv = boost::none) {
        wave::PoseVelBias retval;
        Eigen::MatrixXd J1, J2, J3;
        if (Hv) {
            Hv->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
            J3.resize(3, 3);
        }
        retval.pose = traits<Pose3>::Expmap(v.block<6, 1>(0, 0), J1);
        retval.vel = traits<VelType>::Logmap(v.block<6, 1>(6, 0), J2);
        retval.bias = traits<BiasType>::Logmap(v.block<3, 1>(12, 0), J3);
        if (Hv) {
            Hv->block<6, 6>(0, 0).noalias() = J1;
            Hv->block<6, 6>(6, 6).noalias() = J2;
            Hv->block<3, 3>(12, 12).noalias() = J3;
        }
        return retval;
    }

    static wave::PoseVelBias Compose(const wave::PoseVelBias &m1,
                                     const wave::PoseVelBias &m2,
                                     ChartJacobian H1 = boost::none,
                                     ChartJacobian H2 = boost::none) {
        wave::PoseVelBias retval;
        Eigen::MatrixXd J1, J2, J3, J4, J5, J6;
        if (H1) {
            H1->setZero();
            J1.resize(6, 6);
            J3.resize(6, 6);
            J5.resize(3, 3);
        }
        if (H2) {
            H2->setZero();
            J2.resize(6, 6);
            J4.resize(6, 6);
            J6.resize(3, 3);
        }
        retval.pose = traits<Pose3>::Compose(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<VelType>::Compose(m1.vel, m2.vel, J3, J4);
        retval.bias = traits<BiasType>::Compose(m1.bias, m2.bias, J5, J6);
        if (H1) {
            H1->block<6, 6>(0, 0).noalias() = J1;
            H1->block<6, 6>(6, 6).noalias() = J3;
            H1->block<3, 3>(12, 12).noalias() = J5;
        }
        if (H2) {
            H2->block<6, 6>(0, 0).noalias() = J2;
            H2->block<6, 6>(6, 6).noalias() = J4;
            H2->block<3, 3>(12, 12).noalias() = J6;
        }
        return retval;
    }

    static wave::PoseVelBias Between(const wave::PoseVelBias &m1,
                                     const wave::PoseVelBias &m2,
                                     ChartJacobian H1 = boost::none,
                                     ChartJacobian H2 = boost::none) {
        wave::PoseVelBias retval;
        Eigen::MatrixXd J1, J2, J3, J4, J5, J6;
        if (H1) {
            H1->setZero();
            J1.resize(6, 6);
            J3.resize(6, 6);
            J5.resize(3, 3);
        }
        if (H2) {
            H2->setZero();
            J2.resize(6, 6);
            J4.resize(6, 6);
            J6.resize(3, 3);
        }
        retval.pose = traits<Pose3>::Between(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<VelType>::Between(m1.vel, m2.vel, J3, J4);
        retval.bias = traits<BiasType>::Between(m1.bias, m2.bias, J5, J6);
        if (H1) {
            H1->block<6, 6>(0, 0).noalias() = J1;
            H1->block<6, 6>(6, 6).noalias() = J3;
            H1->block<3, 3>(12, 12).noalias() = J5;
        }
        if (H2) {
            H2->block<6, 6>(0, 0).noalias() = J2;
            H2->block<6, 6>(6, 6).noalias() = J4;
            H2->block<3, 3>(12, 12).noalias() = J6;
        }
        return retval;
    }

    static wave::PoseVelBias Inverse(const wave::PoseVelBias &m,
                                     ChartJacobian H = boost::none) {
        wave::PoseVelBias retval;
        Eigen::MatrixXd J1, J2, J3;
        if (H) {
            H->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
            J3.resize(3, 3);
        }
        retval.pose = traits<Pose3>::Inverse(m.pose, J1);
        retval.vel = traits<VelType>::Inverse(m.vel, J2);
        retval.bias = traits<BiasType>::Inverse(m.bias, J3);
        if (H) {
            H->block<6, 6>(0, 0).noalias() = J1;
            H->block<6, 6>(6, 6).noalias() = J2;
            H->block<3, 3>(12, 12).noalias() = J3;
        }
        return retval;
    }
};
}

#endif  // WAVE_POSE_VEL_BIAS_HPP
