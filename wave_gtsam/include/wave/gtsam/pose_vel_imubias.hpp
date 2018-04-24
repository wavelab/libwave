#ifndef WAVE_POSE_VEL_IMUBIAS_HPP
#define WAVE_POSE_VEL_IMUBIAS_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
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

struct PoseVelImuBias {
    gtsam::Pose3 pose;

    /// Angular followed by linear
    VelType vel;
    // Zero initial bias
    gtsam::imuBias::ConstantBias imu_bias;

    PoseVelImuBias() {
        vel.setZero();
    }

    enum { pose_offset = 0, vel_offset = 6, imu_bias_offset = 12 };
};
}

namespace gtsam {

template <>
struct traits<wave::PoseVelImuBias> {
    typedef lie_group_tag structure_category;

    /**
     * Basic (Testable)
     */

    static void Print(const wave::PoseVelImuBias &m1, const std::string &str = "") {
        traits<Pose3>::Print(m1.pose, str);
        traits<VelType>::Print(m1.vel, str);
        traits<imuBias::ConstantBias>::Print(m1.imu_bias, str);
    }

    static bool Equals(const wave::PoseVelImuBias &m1,
                       const wave::PoseVelImuBias &m2,
                       double tol = 1e-8) {
        if (!traits<Pose3>::Equals(m1.pose, m2.pose, tol)) {
            return false;
        }
        if (!traits<VelType>::Equals(m1.vel, m2.vel, tol)) {
            return false;
        }
        if (!traits<imuBias::ConstantBias>::Equals(m1.imu_bias,
                                                    m2.imu_bias,
                                                    tol)) {
            return false;
        }
        return true;
    }

    /**
     * Manifold
     */

    enum { dimension = 18 };
    static int GetDimension(const wave::PoseVelImuBias &) {
        return dimension;
    }

    typedef wave::PoseVelImuBias ManifoldType;
    typedef Eigen::Matrix<double, dimension, 1> TangentVector;

    // The tangent vector is just stacking all the smaller tangent vectors
    static TangentVector Local(const wave::PoseVelImuBias &origin,
                               const wave::PoseVelImuBias &other) {
        TangentVector retval;
        retval.block<6, 1>(0, 0).noalias() =
          traits<Pose3>::Local(origin.pose, other.pose);
        retval.block<6, 1>(6, 0).noalias() =
          traits<VelType>::Local(origin.vel, other.vel);
        retval.block<6, 1>(12, 0).noalias() =
          traits<imuBias::ConstantBias>::Local(origin.imu_bias,
                                                other.imu_bias);
        return retval;
    }

    static wave::PoseVelImuBias Retract(const wave::PoseVelImuBias &origin,
                                    const TangentVector &v) {
        wave::PoseVelImuBias retval;
        retval.pose = traits<Pose3>::Retract(origin.pose, v.block<6, 1>(0, 0));
        retval.vel = traits<VelType>::Retract(origin.vel, v.block<6, 1>(6, 0));
        retval.imu_bias = traits<imuBias::ConstantBias>::Retract(
                                                    origin.imu_bias,
                                                    v.block<6, 1>(12, 0));
        return retval;
    }

    /**
     * Lie group
     */

    typedef multiplicative_group_tag group_flavor;

    typedef OptionalJacobian<dimension, dimension> ChartJacobian;

    static wave::PoseVelImuBias Identity() {
        return wave::PoseVelImuBias();
    }

    static TangentVector Logmap(const wave::PoseVelImuBias &m,
                                ChartJacobian Hm = boost::none) {
        Eigen::MatrixXd J1, J2, J3;
        TangentVector retval;
        if (Hm) {
            Hm->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
            J3.resize(6, 6);
        }
        retval.block<6, 1>(0, 0).noalias() = traits<Pose3>::Logmap(m.pose, J1);
        retval.block<6, 1>(6, 0).noalias() = traits<VelType>::Logmap(m.vel, J2);
        retval.block<6, 1>(12, 0).noalias() =
                        traits<imuBias::ConstantBias>::Logmap(m.imu_bias, J3);
        if (Hm) {
            Hm->block<6, 6>(0, 0).noalias() = J1;
            Hm->block<6, 6>(6, 6).noalias() = J2;
            Hm->block<6, 6>(12, 12).noalias() = J3;
        }
        return retval;
    }

    static wave::PoseVelImuBias Expmap(const TangentVector &v,
                                ChartJacobian Hv = boost::none) {
        wave::PoseVelImuBias retval;
        Eigen::MatrixXd J1, J2, J3;
        if (Hv) {
            Hv->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
            J3.resize(6, 6);
        }
        retval.pose = traits<Pose3>::Expmap(v.block<6, 1>(0, 0), J1);
        /* Confirm and add comment for why logmap */
        retval.vel = traits<VelType>::Expmap(v.block<6, 1>(6, 0), J2);
        retval.imu_bias =
               traits<imuBias::ConstantBias>::Expmap(v.block<6, 1>(12, 0), J3);
        if (Hv) {
            Hv->block<6, 6>(0, 0).noalias() = J1;
            Hv->block<6, 6>(6, 6).noalias() = J2;
            Hv->block<6, 6>(12, 12).noalias() = J3;
        }
        return retval;
    }

    static wave::PoseVelImuBias Compose(const wave::PoseVelImuBias &m1,
                                         const wave::PoseVelImuBias &m2,
                                         ChartJacobian H1 = boost::none,
                                         ChartJacobian H2 = boost::none) {
        wave::PoseVelImuBias retval;
        Eigen::MatrixXd J1, J2, J3, J4, J5, J6;
        if (H1) {
            H1->setZero();
            J1.resize(6, 6);
            J3.resize(6, 6);
            J5.resize(6, 6);
        }
        if (H2) {
            H2->setZero();
            J2.resize(6, 6);
            J4.resize(6, 6);
            J6.resize(6, 6);
        }
        retval.pose = traits<Pose3>::Compose(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<VelType>::Compose(m1.vel, m2.vel, J3, J4);
        retval.imu_bias = traits<imuBias::ConstantBias>::Compose(m1.imu_bias,
                                                            m2.imu_bias,
                                                            J5, J6);
        if (H1) {
            H1->block<6, 6>(0, 0).noalias() = J1;
            H1->block<6, 6>(6, 6).noalias() = J3;
            H1->block<6, 6>(12, 12).noalias() = J5;
        }
        if (H2) {
            H2->block<6, 6>(0, 0).noalias() = J2;
            H2->block<6, 6>(6, 6).noalias() = J4;
            H2->block<6, 6>(12, 12).noalias() = J6;
        }
        return retval;
    }

    static wave::PoseVelImuBias Between(const wave::PoseVelImuBias &m1,
                                     const wave::PoseVelImuBias &m2,
                                     ChartJacobian H1 = boost::none,
                                     ChartJacobian H2 = boost::none) {
        wave::PoseVelImuBias retval;
        Eigen::MatrixXd J1, J2, J3, J4, J5, J6;
        if (H1) {
            H1->setZero();
            J1.resize(6, 6);
            J3.resize(6, 6);
            J5.resize(6, 6);
        }
        if (H2) {
            H2->setZero();
            J2.resize(6, 6);
            J4.resize(6, 6);
            J6.resize(6, 6);
        }
        retval.pose = traits<Pose3>::Between(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<VelType>::Between(m1.vel, m2.vel, J3, J4);
        retval.imu_bias = traits<imuBias::ConstantBias>::Between(m1.imu_bias,
                                                            m2.imu_bias,
                                                            J5, J6);
        if (H1) {
            H1->block<6, 6>(0, 0).noalias() = J1;
            H1->block<6, 6>(6, 6).noalias() = J3;
            H1->block<6, 6>(12, 12).noalias() = J5;
        }
        if (H2) {
            H2->block<6, 6>(0, 0).noalias() = J2;
            H2->block<6, 6>(6, 6).noalias() = J4;
            H2->block<6, 6>(12, 12).noalias() = J6;
        }
        return retval;
    }

    static wave::PoseVelImuBias Inverse(const wave::PoseVelImuBias &m,
                                 ChartJacobian H = boost::none) {
        wave::PoseVelImuBias retval;
        Eigen::MatrixXd J1, J2, J3;
        if (H) {
            H->setZero();
            J1.resize(6, 6);
            J2.resize(6, 6);
            J3.resize(6, 6);
        }
        retval.pose = traits<Pose3>::Inverse(m.pose, J1);
        retval.vel = traits<VelType>::Inverse(m.vel, J2);
        retval.imu_bias = 
                traits<imuBias::ConstantBias>::Inverse(m.imu_bias, J3);
        if (H) {
            H->block<6, 6>(0, 0).noalias() = J1;
            H->block<6, 6>(6, 6).noalias() = J2;
            H->block<6, 6>(12, 12).noalias() = J3;
        }
        return retval;
    }
};
}

#endif  // WAVE_POSE_VEL_IMUBIAS_HPP
