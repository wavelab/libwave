#ifndef WAVE_POSE_VEL_ACC_BIAS_HPP
#define WAVE_POSE_VEL_ACC_BIAS_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

/**
 * This implements the traits required to optimize with gtsam
 * This state is combined pose, velocity, acceleration, and some biases.
 * Follows implementation example of https://github.com/dongjing3309/gtsam-examples/blob/master/cpp/Point2c.h
 * but is a bit different because all the members have traits already defined
 */

namespace wave {

//todo: Figure out how to template dimension of bias
struct PoseVelAccBias {
    gtsam::Pose3 pose;
    Eigen::Matrix<double, 6, 1> vel;
    Eigen::Matrix<double, 6, 1> acc;
    Eigen::Matrix<double, 3, 1> bias;

    PoseVelAccBias() {
        vel.setZero();
        acc.setZero();
        bias.setZero();
    }
};

}

namespace gtsam {

template <>
struct traits<wave::PoseVelAccBias> {

    typedef lie_group_tag structure_category;

    /**
     * Basic (Testable)
     */

    static void Print(const wave::PoseVelAccBias& m1, const std::string& str = "") {
        traits<Pose3>::Print(m1.pose, str);
        traits<decltype(m1.vel)>::Print(m1.vel, str);
        traits<decltype(m1.acc)>::Print(m1.acc, str);
        traits<decltype(m1.bias)>::Print(m1.bias, str);
    }

    static bool Equals(const wave::PoseVelAccBias& m1, const wave::PoseVelAccBias& m2, double tol = 1e-8) {
        if(!traits<Pose3>::Equals(m1.pose, m2.pose, tol)) {
            return false;
        }
        if(!traits<decltype(m1.vel)>::Equals(m1.vel, m2.vel, tol)) {
            return false;
        }
        if(!traits<decltype(m1.acc)>::Equals(m1.acc, m2.acc, tol)) {
            return false;
        }
        if(!traits<decltype(m1.bias)>::Equals(m1.bias, m2.bias, tol)) {
            return false;
        }
        return true;
    }

    /**
     * Manifold
     */

    enum { dimension = 21 };
    static int GetDimension(const wave::PoseVelAccBias&) { return dimension; }

    typedef wave::PoseVelAccBias ManifoldType;
    typedef Eigen::Matrix<double, dimension, 1> TangentVector;

    // The tangent vector is just stacking all the smaller tangent vectors
    static TangentVector Local(const wave::PoseVelAccBias& origin, const wave::PoseVelAccBias& other) {
        TangentVector retval;
        retval.block<6,1>(0,0).noalias() = traits<Pose3>::Local(origin.pose, other.pose);
        retval.block<6,1>(6,0).noalias() = traits<decltype(origin.vel)>::Local(origin.vel, other.vel);
        retval.block<6,1>(12,0).noalias() = traits<decltype(origin.acc)>::Local(origin.acc, other.acc);
        retval.block<3,1>(18,0).noalias() = traits<decltype(origin.bias)>::Local(origin.bias, other.bias);
        return retval;
    }

    static wave::PoseVelAccBias Retract(const wave::PoseVelAccBias& origin, const TangentVector& v) {
        wave::PoseVelAccBias retval;
        retval.pose = traits<Pose3>::Retract(origin.pose, v.block<6,1>(0,0).);
        retval.vel = traits<decltype(retval.vel)>::Retract(origin.vel, v.block<6,1>(6,0).);
        retval.acc = traits<decltype(retval.acc)>::Retract(origin.acc, v.block<6,1>(12,0).);
        retval.bias = traits<decltype(retval.bias)>::Retract(origin.bias, v.block<3,1>(18,0).);
        return retval;
    }

    /**
     * Lie group
     */

    typedef multiplicative_group_tag group_flavor;

    typedef OptionalJacobian<dimension, dimension> ChartJacobian;

    static wave::PoseVelAccBias Identity() {
        return wave::PoseVelAccBias();
    }

    static TangentVector Logmap(const wave::PoseVelAccBias& m, ChartJacobian Hm = boost::none) {
        TangentVector retval;
        traits<Pose3>::ChartJacobian J1;
        traits<decltype(m.vel)>::ChartJacobian J2;
        traits<decltype(m.acc)>::ChartJacobian J3;
        traits<decltype(m.bias)>::ChartJacobian J4;
        if (Hm) {
            Hm->resize(dimension, dimension);
            Hm->setZero();
            retval.block<6,1>(0,0).noalias() = traits<Pose3>::Logmap(m.pose, J1);
            retval.block<6,1>(6,0).noalias() = traits<decltype(m.vel)>::Logmap(m.vel, J2);
            retval.block<6,1>(12,0).noalias() = traits<decltype(m.acc)>::Logmap(m.acc, J3);
            retval.block<3,1>(18,0).noalias() = traits<decltype(m.bias)>::Logmap(m.bias, J4);

            Hm->block<6,6>(0,0).noalias() = *J1;
            Hm->block<6,6>(6,6).noalias() = *J2;
            Hm->block<6,6>(12,12).noalias() = *J2;
            Hm->block<3,3>(18,18).noalias() = *J4;
        } else {
            retval.block<6,1>(0,0).noalias() = traits<Pose3>::Logmap(m.pose, boost::none);
            retval.block<6,1>(6,0).noalias() = traits<decltype(m.vel)>::Logmap(m.vel, boost::none);
            retval.block<6,1>(12,0).noalias() = traits<decltype(m.acc)>::Logmap(m.acc, boost::none);
            retval.block<3,1>(18,0).noalias() = traits<decltype(m.bias)>::Logmap(m.bias, boost::none);
        }
        return retval;
    }

    static wave::PoseVelAccBias Expmap(const TangentVector& v, ChartJacobian Hv = boost::none) {
        wave::PoseVelAccBias retval;
        traits<Pose3>::ChartJacobian J1;
        traits<decltype(m.vel)>::ChartJacobian J2;
        traits<decltype(m.acc)>::ChartJacobian J3;
        traits<decltype(m.bias)>::ChartJacobian J4;
        if (Hv) {
            Hv->resize(dimension, dimension);
            Hv->setZero();
            retval.pose = traits<Pose3>::Expmap(v.block<6,1>(0,0)., J1);
            retval.vel = traits<decltype(m.vel)>::Logmap(v.block<6,1>(6,0)., J2);
            retval.acc = traits<decltype(m.acc)>::Logmap(v.block<6,1>(12,0)., J3);
            retval.bias = traits<decltype(m.bias)>::Logmap(v.block<3,1>(18,0)., J4);

            Hv->block<6,6>(0,0).noalias() = *J1;
            Hv->block<6,6>(6,6).noalias() = *J2;
            Hv->block<6,6>(12,12).noalias() = *J3;
            Hv->block<3,3>(18,18).noalias() = *J4;
        } else {
            retval.pose = traits<Pose3>::Expmap(v.block<6,1>(0,0)., boost::none);
            retval.vel = traits<decltype(m.vel)>::Logmap(v.block<6,1>(6,0)., boost::none);
            retval.acc = traits<decltype(m.acc)>::Logmap(v.block<6,1>(12,0)., boost::none);
            retval.bias = traits<decltype(m.bias)>::Logmap(v.block<3,1>(18,0)., boost::none);
        }
        return retval;
    }

    static wave::PoseVelAccBias Compose(const wave::PoseVelAccBias& m1, const wave::PoseVelAccBias& m2, ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
        wave::PoseVelAccBias retval;
        traits<Pose3>::ChartJacobian J1, J2;
        traits<decltype(m.vel)>::ChartJacobian J3, J4;
        traits<decltype(m.acc)>::ChartJacobian J5, J6;
        traits<decltype(m.bias)>::ChartJacobian J7, J8;

        retval.pose = traits<Pose3>::Compose(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<decltype(m.vel)>::Compose(m1.vel, m2.vel, J3, J4);
        retval.acc = traits<decltype(m.acc)>::Compose(m1.acc, m2.acc, J5, J6);
        retval.bias = traits<decltype(m.bias)>::Compose(m1.bias, m2.bias, J7, J8);

        if(H1) {
            H1->resize(dimension, dimension);
            H1->setZero();
            H1->block<6,6>(0,0).noalias() = *J1;
            H1->block<6,6>(6,6).noalias() = *J3;
            H1->block<6,6>(12,12).noalias() = *J5;
            H1->block<3,3>(18,18).noalias() = *J7;
        }
        if(H2) {
            H2->resize(dimension, dimension);
            H2->setZero();
            H2->block<6,6>(0,0).noalias() = *J2;
            H2->block<6,6>(6,6).noalias() = *J4;
            H2->block<6,6>(12,12).noalias() = *J6;
            H2->block<3,3>(18,18).noalias() = *J8;
        }
        return retval;
    }

    static wave::PoseVelAccBias Between(const wave::PoseVelAccBias& m1, const wave::PoseVelAccBias& m2, ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
        wave::PoseVelAccBias retval;
        traits<Pose3>::ChartJacobian J1, J2;
        traits<decltype(m.vel)>::ChartJacobian J3, J4;
        traits<decltype(m.acc)>::ChartJacobian J5, J6;
        traits<decltype(m.bias)>::ChartJacobian J7, J8;

        retval.pose = traits<Pose3>::Between(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<decltype(m.vel)>::Between(m1.vel, m2.vel, J3, J4);
        retval.acc = traits<decltype(m.acc)>::Between(m1.acc, m2.acc, J5, J6);
        retval.bias = traits<decltype(m.bias)>::Between(m1.bias, m2.bias, J7, J8);

        if(H1) {
            H1->resize(dimension, dimension);
            H1->setZero();
            H1->block<6,6>(0,0).noalias() = *J1;
            H1->block<6,6>(6,6).noalias() = *J3;
            H1->block<6,6>(12,12).noalias() = *J5;
            H1->block<3,3>(18,18).noalias() = *J7;
        }
        if(H2) {
            H2->resize(dimension, dimension);
            H2->setZero();
            H2->block<6,6>(0,0).noalias() = *J2;
            H2->block<6,6>(6,6).noalias() = *J4;
            H2->block<6,6>(12,12).noalias() = *J6;
            H2->block<3,3>(18,18).noalias() = *J8;
        }
        return retval;
    }

    static wave::PoseVelAccBias Inverse(const wave::PoseVelAccBias& m, ChartJacobian H = boost::none) {
        wave::PoseVelAccBias retval;
        traits<Pose3>::ChartJacobian J1;
        traits<decltype(m.vel)>::ChartJacobian J2;
        traits<decltype(m.acc)>::ChartJacobian J3;
        traits<decltype(m.bias)>::ChartJacobian J4;

        retval.pose = traits<Pose3>::Inverse(m.pose, J1);
        retval.vel = traits<decltype(m.vel)>::Inverse(m.vel, J2);
        retval.acc = traits<decltype(m.acc)>::Inverse(m.acc, J3);
        retval.bias = traits<decltype(m.bias)>::Inverse(m.bias, J4);
        if(H) {
            H->resize(dimension, dimension);
            H->setZero();
            H->block<6,6>(0,0).noalias() = *J1;
            H->block<6,6>(6,6).noalias() = *J2;
            H->block<6,6>(12,12).noalias() = *J3;
            H->block<3,3>(18,18).noalias() = *J4;
        }
        return retval;
    }

};

}

namespace wave {

PoseVelAccBias operator*(const PoseVelAccBias& m1, const PoseVelAccBias& m2) {
    wave::PoseVelAccBias retval;
    retval.pose = m1.pose * m2.pose;
    retval.vel = m1.vel + m2.vel;
    retval.acc = m1.acc + m2.acc;
    retval.bias = m1.bias + m2.bias;
}

}

#endif //WAVE_POSE_VEL_ACC_BIAS_HPP
