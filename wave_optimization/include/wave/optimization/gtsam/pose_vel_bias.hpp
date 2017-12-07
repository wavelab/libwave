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
 * Follows implementation example of https://github.com/dongjing3309/gtsam-examples/blob/master/cpp/Point2c.h
 * but is a bit different because all the members have traits already defined
 */

namespace wave {

//todo: Figure out how to template dimension of bias
struct PoseVelBias {
    gtsam::Pose3 pose;
    Eigen::Matrix<double, 6, 1> vel;
    Eigen::Matrix<double, 3, 1> bias;

    PoseVelBias() {
        vel.setZero();
        bias.setZero();
    }
};

}

namespace gtsam {

template <>
struct traits<wave::PoseVelBias> {

    typedef lie_group_tag structure_category;

    /**
     * Basic (Testable)
     */

    static void Print(const wave::PoseVelBias& m1, const std::string& str = "") {
        traits<Pose3>::Print(m1.pose, str);
        traits<decltype(m1.vel)>::Print(m1.vel, str);
        traits<decltype(m1.bias)>::Print(m1.bias, str);
    }

    static bool Equals(const wave::PoseVelBias& m1, const wave::PoseVelBias& m2, double tol = 1e-8) {
        if(!traits<Pose3>::Equals(m1.pose, m2.pose, tol)) {
            return false;
        }
        if(!traits<decltype(m1.vel)>::Equals(m1.vel, m2.vel, tol)) {
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

    enum { dimension = 15 };
    static int GetDimension(const wave::PoseVelBias&) { return dimension; }

    typedef wave::PoseVelBias ManifoldType;
    typedef Eigen::Matrix<double, dimension, 1> TangentVector;

    // The tangent vector is just stacking all the smaller tangent vectors
    static TangentVector Local(const wave::PoseVelBias& origin, const wave::PoseVelBias& other) {
        TangentVector retval;
        retval.block<6,1>(0,0).noalias() = traits<Pose3>::Local(origin.pose, other.pose);
        retval.block<6,1>(6,0).noalias() = traits<decltype(origin.vel)>::Local(origin.vel, other.vel);
        retval.block<3,1>(12,0).noalias() = traits<decltype(origin.bias)>::Local(origin.bias, other.bias);
        return retval;
    }

    static wave::PoseVelBias Retract(const wave::PoseVelBias& origin, const TangentVector& v) {
        wave::PoseVelBias retval;
        retval.pose = traits<Pose3>::Retract(origin.pose, v.block<6,1>(0,0).noalias());
        retval.vel = traits<decltype(retval.vel)>::Retract(origin.vel, v.block<6,1>(6,0).noalias());
        retval.bias = traits<decltype(retval.bias)>::Retract(origin.bias, v.block<3,1>(12,0).noalias());
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

    static TangentVector Logmap(const wave::PoseVelBias& m, ChartJacobian Hm = boost::none) {
        TangentVector retval;
        traits<Pose3>::ChartJacobian J1;
        traits<decltype(m.vel)>::ChartJacobian J2;
        traits<decltype(m.bias)>::ChartJacobian J3;
        if (Hm) {
            Hm->resize(dimension, dimension);
            Hm->setZero();
            retval.block<6,1>(0,0).noalias() = traits<Pose3>::Logmap(m.pose, J1);
            retval.block<6,1>(6,0).noalias() = traits<decltype(m.vel)>::Logmap(m.vel, J2);
            retval.block<3,1>(12,0).noalias() = traits<decltype(m.bias)>::Logmap(m.bias, J3);

            Hm->block<6,6>(0,0).noalias() = *J1;
            Hm->block<6,6>(6,6).noalias() = *J2;
            Hm->block<3,3>(12,12).noalias() = *J3;
        } else {
            retval.block<6,1>(0,0).noalias() = traits<Pose3>::Logmap(m.pose, boost::none);
            retval.block<6,1>(6,0).noalias() = traits<decltype(m.vel)>::Logmap(m.vel, boost::none);
            retval.block<3,1>(12,0).noalias() = traits<decltype(m.bias)>::Logmap(m.bias, boost::none);
        }
        return retval;
    }

    static wave::PoseVelBias Expmap(const TangentVector& v, ChartJacobian Hv = boost::none) {
        wave::PoseVelBias retval;
        traits<Pose3>::ChartJacobian J1;
        traits<decltype(m.vel)>::ChartJacobian J2;
        traits<decltype(m.bias)>::ChartJacobian J3;
        if (Hv) {
            Hv->resize(dimension, dimension);
            Hv->setZero();
            retval.pose = traits<Pose3>::Expmap(v.block<6,1>(0,0).noalias(), J1);
            retval.vel = traits<decltype(m.vel)>::Logmap(v.block<6,1>(6,0).noalias(), J2);
            retval.bias = traits<decltype(m.bias)>::Logmap(v.block<3,1>(12,0).noalias(), J3);

            Hv->block<6,6>(0,0).noalias() = *J1;
            Hv->block<6,6>(6,6).noalias() = *J2;
            Hv->block<3,3>(12,12).noalias() = *J3;
        } else {
            retval.pose = traits<Pose3>::Expmap(v.block<6,1>(0,0).noalias(), boost::none);
            retval.vel = traits<decltype(m.vel)>::Logmap(v.block<6,1>(6,0).noalias(), boost::none);
            retval.bias = traits<decltype(m.bias)>::Logmap(v.block<3,1>(12,0).noalias(), boost::none);
        }
        return retval;
    }

    static wave::PoseVelBias Compose(const wave::PoseVelBias& m1, const wave::PoseVelBias& m2, ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
        wave::PoseVelBias retval;
        traits<Pose3>::ChartJacobian J1, J2;
        traits<decltype(m.vel)>::ChartJacobian J3, J4;
        traits<decltype(m.bias)>::ChartJacobian J5, J6;

        retval.pose = traits<Pose3>::Compose(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<decltype(m.vel)>::Compose(m1.vel, m2.vel, J3, J4);
        retval.bias = traits<decltype(m.bias)>::Compose(m1.bias, m2.bias, J5, J6);

        if(H1) {
            H1->resize(dimension, dimension);
            H1->setZero();
            H1->block<6,6>(0,0).noalias() = *J1;
            H1->block<6,6>(6,6).noalias() = *J3;
            H1->block<3,3>(12,12).noalias() = *J5;
        }
        if(H2) {
            H2->resize(dimension, dimension);
            H2->setZero();
            H2->block<6,6>(0,0).noalias() = *J2;
            H2->block<6,6>(6,6).noalias() = *J4;
            H2->block<3,3>(12,12).noalias() = *J6;
        }
        return retval;
    }

    static wave::PoseVelBias Between(const wave::PoseVelBias& m1, const wave::PoseVelBias& m2, ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
        wave::PoseVelBias retval;
        traits<Pose3>::ChartJacobian J1, J2;
        traits<decltype(m.vel)>::ChartJacobian J3, J4;
        traits<decltype(m.bias)>::ChartJacobian J5, J6;

        retval.pose = traits<Pose3>::Between(m1.pose, m2.pose, J1, J2);
        retval.vel = traits<decltype(m.vel)>::Between(m1.vel, m2.vel, J3, J4);
        retval.bias = traits<decltype(m.bias)>::Between(m1.bias, m2.bias, J5, J6);

        if(H1) {
            H1->resize(dimension, dimension);
            H1->setZero();
            H1->block<6,6>(0,0).noalias() = *J1;
            H1->block<6,6>(6,6).noalias() = *J3;
            H1->block<3,3>(12,12).noalias() = *J5;
        }
        if(H2) {
            H2->resize(dimension, dimension);
            H2->setZero();
            H2->block<6,6>(0,0).noalias() = *J2;
            H2->block<6,6>(6,6).noalias() = *J4;
            H2->block<3,3>(12,12).noalias() = *J6;
        }
        return retval;
    }

    static wave::PoseVelBias Inverse(const wave::PoseVelBias& m, ChartJacobian H = boost::none) {
        wave::PoseVelBias retval;
        traits<Pose3>::ChartJacobian J1;
        traits<decltype(m.vel)>::ChartJacobian J2;
        traits<decltype(m.bias)>::ChartJacobian J3;

        retval.pose = traits<Pose3>::Inverse(m.pose, J1);
        retval.vel = traits<decltype(m.vel)>::Inverse(m.vel, J2);
        retval.bias = traits<decltype(m.bias)>::Inverse(m.bias, J3);
        if(H) {
            H->resize(dimension, dimension);
            H->setZero();
            H->block<6,6>(0,0).noalias() = *J1;
            H->block<6,6>(6,6).noalias() = *J2;
            H->block<3,3>(12,12).noalias() = *J3;
        }
        return retval;
    }

};

}

namespace wave {

PoseVelBias operator*(const PoseVelBias& m1, const PoseVelBias& m2) {
    wave::PoseVelBias retval;
    retval.pose = m1.pose * m2.pose;
    retval.vel = m1.vel + m2.vel;
    retval.bias = m1.bias + m2.bias;
}

}

#endif //WAVE_POSE_VEL_BIAS_HPP
