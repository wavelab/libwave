#include "wave/utils/config.hpp"
#include "wave/matching/icp.hpp"

namespace wave {

ICPMatcher::ICPMatcher(float res, const std::string &config_path) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (res > 0) {
        this->resolution = res;
        this->filter.setLeafSize(res, res, res);
    } else {
        this->resolution = -1;
    }

    ConfigParser parser;
    double max_corr, t_eps, fit_eps;
    int max_iter, estimate_method_int;
    parser.addParam("icp.max_corr", &max_corr);
    parser.addParam("icp.max_iter", &max_iter);
    parser.addParam("icp.t_eps", &t_eps);
    parser.addParam("icp.fit_eps", &fit_eps);
    parser.addParam("icp.lidar_ang_covar", &(this->lidar_ang_covar));
    parser.addParam("icp.lidar_lin_covar", &(this->lidar_lin_covar));
    parser.addParam("icp.covar_estimator", &estimate_method_int);

    if (parser.load(config_path) != 0) {
        ConfigException config_exception;
        throw config_exception;
    }

    this->estimate_method =
      static_cast<ICPMatcher::covar_method>(estimate_method_int);
    this->icp.setMaxCorrespondenceDistance(max_corr);
    this->icp.setMaximumIterations(max_iter);
    this->icp.setTransformationEpsilon(t_eps);
    this->icp.setEuclideanFitnessEpsilon(fit_eps);
}

ICPMatcher::~ICPMatcher() {
    if (this->ref) {
        this->ref.reset();
    }
    if (this->target) {
        this->target.reset();
    }
    if (this->final) {
        this->final.reset();
    }
}

void ICPMatcher::setRef(const PCLPointCloud &ref) {
    if (this->resolution > 0) {
        this->filter.setInputCloud(ref);
        this->filter.filter(*(this->ref));
    } else {
        this->ref = ref;
    }
    this->icp.setInputSource(this->ref);
}

void ICPMatcher::setTarget(const PCLPointCloud &target) {
    if (this->resolution > 0) {
        this->filter.setInputCloud(target);
        this->filter.filter(*(this->target));
    } else {
        this->target = target;
    }
    this->icp.setInputTarget(this->target);
}

bool ICPMatcher::match() {
    this->icp.align(*(this->final));
    if (this->icp.hasConverged()) {
        this->result.matrix() = icp.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}

void ICPMatcher::estimateInfo() {
    switch (this->estimate_method) {
        case ICPMatcher::covar_method::LUM: this->estimateLUM();
        case ICPMatcher::covar_method::CENSI: this->estimateCensi();
        default: return;
    }
}

// Taken from the Lu and Milios matcher in PCL
void ICPMatcher::estimateLUM() {
    if (this->icp.hasConverged()) {
        auto list = this->icp.correspondences_.get();
        Mat6 MM = Mat6::Zero();
        Vec6 MZ = Vec6::Zero();
        std::vector<Eigen::Vector3f> corrs_aver;
        std::vector<Eigen::Vector3f> corrs_diff;


        int numCorr = 0;
        for (auto it = list->begin(); it != list->end(); ++it) {
            if (it->index_match > -1) {
                corrs_aver.push_back(Eigen::Vector3f(
                  0.5f * (this->ref->points[it->index_query].x +
                          this->target->points[it->index_match].x),
                  0.5f * (this->ref->points[it->index_query].y +
                          this->target->points[it->index_match].y),
                  0.5f * (this->ref->points[it->index_query].z +
                          this->target->points[it->index_match].z)));
                corrs_diff.push_back(
                  Eigen::Vector3f(this->ref->points[it->index_query].x -
                                    this->target->points[it->index_match].x,
                                  this->ref->points[it->index_query].y -
                                    this->target->points[it->index_match].y,
                                  this->ref->points[it->index_query].z -
                                    this->target->points[it->index_match].z));
                numCorr++;
            }
        }

        for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
        {
            // Fast computation of summation elements of M'M
            MM(0, 4) -= corrs_aver[ci](1);
            MM(0, 5) += corrs_aver[ci](2);
            MM(1, 3) -= corrs_aver[ci](2);
            MM(1, 4) += corrs_aver[ci](0);
            MM(2, 3) += corrs_aver[ci](1);
            MM(2, 5) -= corrs_aver[ci](0);
            MM(3, 4) -= corrs_aver[ci](0) * corrs_aver[ci](2);
            MM(3, 5) -= corrs_aver[ci](0) * corrs_aver[ci](1);
            MM(4, 5) -= corrs_aver[ci](1) * corrs_aver[ci](2);
            MM(3, 3) += corrs_aver[ci](1) * corrs_aver[ci](1) +
                        corrs_aver[ci](2) * corrs_aver[ci](2);
            MM(4, 4) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                        corrs_aver[ci](1) * corrs_aver[ci](1);
            MM(5, 5) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                        corrs_aver[ci](2) * corrs_aver[ci](2);

            // Fast computation of M'Z
            MZ(0) += corrs_diff[ci](0);
            MZ(1) += corrs_diff[ci](1);
            MZ(2) += corrs_diff[ci](2);
            MZ(3) += corrs_aver[ci](1) * corrs_diff[ci](2) -
                     corrs_aver[ci](2) * corrs_diff[ci](1);
            MZ(4) += corrs_aver[ci](0) * corrs_diff[ci](1) -
                     corrs_aver[ci](1) * corrs_diff[ci](0);
            MZ(5) += corrs_aver[ci](2) * corrs_diff[ci](0) -
                     corrs_aver[ci](0) * corrs_diff[ci](2);
        }
        // Remaining elements of M'M
        MM(0, 0) = MM(1, 1) = MM(2, 2) = static_cast<float>(numCorr);
        MM(4, 0) = MM(0, 4);
        MM(5, 0) = MM(0, 5);
        MM(3, 1) = MM(1, 3);
        MM(4, 1) = MM(1, 4);
        MM(3, 2) = MM(2, 3);
        MM(5, 2) = MM(2, 5);
        MM(4, 3) = MM(3, 4);
        MM(5, 3) = MM(3, 5);
        MM(5, 4) = MM(4, 5);

        // Compute pose difference estimation
        Vec6 D = static_cast<Vec6>(MM.inverse() * MZ);

        // Compute s^2
        float ss = 0.0f;
        for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
        {
            ss += static_cast<float>(
              pow(corrs_diff[ci](0) - (D(0) + corrs_aver[ci](2) * D(5) -
                                       corrs_aver[ci](1) * D(4)),
                  2.0f) +
              pow(corrs_diff[ci](1) - (D(1) + corrs_aver[ci](0) * D(4) -
                                       corrs_aver[ci](2) * D(3)),
                  2.0f) +
              pow(corrs_diff[ci](2) - (D(2) + corrs_aver[ci](1) * D(3) -
                                       corrs_aver[ci](0) * D(5)),
                  2.0f));
        }

        // When reaching the limitations of computation due to linearization
        if (ss < 0.0000000000001 || !pcl_isfinite(ss)) {
            this->information = Mat6::Identity();
            return;
        }

        this->information = MM * (1.0f / ss);
    }
}

// This is an implementation of the Haralick or Censi covariance approximation
// for ICP
// The core idea behind this is that the covariance of the cost f'n J wrt
// optimization variable x is
// cov(x) ~= (d2J/dx2)^-1*(d2J/dzdx)*cov(z)*(d2J/dzdx)'*(d2J/dx2)^-1

// Idea was taken from
// https://censi.science/pub/research/2007-icra-icpcov.pdf

// This is an implementation for euler angles, what is below is a cleaned up
// version of
// http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7153246

void ICPMatcher::estimateCensi() {
    if (this->icp.hasConverged()) {
        const auto eulers = this->result.rotation().eulerAngles(0, 1, 2);
        const auto translation = this->result.translation();
        // set up aliases to shrink following lines
        const double &X1 = translation.x(), X2 = translation.y(),
                     X3 = translation.z();
        // precompute trig quantities
        double cr, sr, cp, sp, cy, sy;
        // r = roll, p = pitch, y = yaw, c = cos, s = sine
        cr = cos(eulers[0]);
        sr = sin(eulers[0]);
        cp = cos(eulers[1]);
        sp = sin(eulers[1]);
        cy = cos(eulers[2]);
        sy = sin(eulers[2]);

        Eigen::DiagonalMatrix<double, 6> sphere_cov;
        sphere_cov.diagonal() << this->lidar_lin_covar, this->lidar_ang_covar,
          this->lidar_ang_covar, this->lidar_lin_covar, this->lidar_ang_covar,
          this->lidar_ang_covar;
        Eigen::MatrixXd cov_Z(Eigen::MatrixXd::Zero(6, 6));
        Eigen::MatrixXd j(Eigen::MatrixXd::Zero(6, 6));
        double az, br, rg;  // azimuth, bearing and range

        // The ordering for partials is x, y, z, rotx, roty, rotz
        // This is a symmetric matrix, so only need to fill out the upper
        // triangular portion
        Eigen::MatrixXd d2J_dX2(Eigen::MatrixXd::Zero(6, 6));

        // To hold running total of d2J_dZdX*cov(z)*d2J_dZdX'
        Eigen::MatrixXd middle(Eigen::MatrixXd::Zero(6, 6));

        // Gets overwritten each loop
        Eigen::MatrixXd d2J_dZdX(Eigen::MatrixXd::Zero(6, 6));
        d2J_dZdX(3, 0) = -2;
        d2J_dZdX(4, 1) = -2;
        d2J_dZdX(5, 2) = -2;

        auto list = this->icp.correspondences_.get();
        for (auto it = list->begin(); it != list->end(); ++it) {
            if (it->index_match > -1) {
                // it is -1 if there is no match in the target cloud
                // set up some aliases to make following lines more compact
                const float &Z1 = (this->target->points[it->index_match].x),
                            Z2 = (this->target->points[it->index_match].y),
                            Z3 = (this->target->points[it->index_match].z),
                            Z4 = (this->ref->points[it->index_query].x),
                            Z5 = (this->ref->points[it->index_query].y),
                            Z6 = (this->ref->points[it->index_query].z);

                rg = std::sqrt(Z1 * Z1 + Z2 * Z2 + Z3 * Z3);
                br = std::atan2(Z2, Z1);
                az = std::atan(Z3 / std::sqrt(Z1 * Z1 + Z2 * Z2));
                j(0, 0) = cos(br) * sin(az);
                j(1, 0) = sin(br) * sin(az);
                j(2, 0) = cos(az);
                j(0, 1) = -rg * sin(br) * sin(az);
                j(1, 1) = rg * cos(br) * sin(az);
                j(0, 2) = rg * cos(br) * cos(az);
                j(1, 2) = rg * cos(az) * sin(br);
                j(2, 2) = -rg * sin(az);
                rg = std::sqrt(Z4 * Z4 + Z5 * Z5 + Z6 * Z6);
                br = std::atan2(Z5, Z4);
                az = std::atan(Z6 / std::sqrt(Z4 * Z4 + Z5 * Z5));
                j(3, 3) = cos(br) * sin(az);
                j(4, 3) = sin(br) * sin(az);
                j(5, 3) = cos(az);
                j(3, 4) = -rg * sin(br) * sin(az);
                j(4, 4) = rg * cos(br) * sin(az);
                j(3, 5) = rg * cos(br) * cos(az);
                j(4, 5) = rg * cos(az) * sin(br);
                j(5, 5) = -rg * sin(az);
                cov_Z = j * sphere_cov.derived() * j.transpose();

                // clang-format off

                // coordinate transform jacobian. Order is range, bearing,
                // azimuth for first and 2nd point
                // [ cos(S2)*sin(S3), -S1*sin(S2)*sin(S3),   S1*cos(S2)*cos(S3),               0,                0,                  0]
                // [ sin(S2)*sin(S3),  S1*cos(S2)*sin(S3),   S1*cos(S3)*sin(S2),               0,                0,                  0]
                // [         cos(S3),                   0,          -S1*sin(S3),               0,                0,                  0]
                // [               0,                   0,                    0, cos(S5)*sin(S6), -S4*sin(S5)*sin(S6), S4*cos(S5)*cos(S6)]
                // [               0,                   0,                   0, sin(S5)*sin(S6),  S4*cos(S5)*sin(S6),  S4*cos(S6)*sin(S5)]
                // [               0,                   0,                  0,         cos(S6),                   0,       -S4*sin(S6)]

                // d2J_dx2

                d2J_dX2(0, 0) += 2;
                d2J_dX2(1, 1) += 2;
                d2J_dX2(2, 2) += 2;

                d2J_dX2(0, 3) += 2 * Z2 * (sr * sy + cr * cy * sp) + 2 * Z3 * (cr * sy - cy * sr * sp);
                d2J_dX2(1, 3) += -2 * Z2 * (cy * sr - cr * sp * sy) - 2 * Z3 * (cr * cy + sr * sp * sy);
                d2J_dX2(2, 3) += 2 * cp * (Z2 * cr - Z3 * sr);
                d2J_dX2(3, 3) += (2 * Z2 * (cr * sy - cy * sr * sp) - 2 * Z3 * (sr * sy + cr * cy * sp)) * (X1 - Z4 - Z2 * (cr * sy - cy * sr * sp) +
                     Z3 * (sr * sy + cr * cy * sp) + Z1 * cp * cy) - (2 * Z2 * (cr * cy + sr * sp * sy) - 2 * Z3 * (cy * sr - cr * sp * sy)) *
                    (X2 - Z5 + Z2 * (cr * cy + sr * sp * sy) - Z3 * (cy * sr - cr * sp * sy) + Z1 * cp * sy) - (2 * Z3 * cr * cp + 2 * Z2 * cp * sr) *
                    (X3 - Z6 - Z1 * sp + Z3 * cr * cp + Z2 * cp * sr) + (Z2 * (sr * sy + cr * cy * sp) + Z3 * (cr * sy - cy * sr * sp)) *
                    (2 * Z2 * (sr * sy + cr * cy * sp) + 2 * Z3 * (cr * sy - cy * sr * sp)) + (Z2 * (cy * sr - cr * sp * sy) +
                    Z3 * (cr * cy + sr * sp * sy)) * (2 * Z2 * (cy * sr - cr * sp * sy) + 2 * Z3 * (cr * cy + sr * sp * sy)) +
                    (Z2 * cr * cp - Z3 * cp * sr) * (2 * Z2 * cr * cp - 2 * Z3 * cp * sr);

                d2J_dX2(0, 4) += 2 * cy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dX2(1, 4) += 2 * sy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dX2(2, 4) += -2 * Z1 * cp - 2 * Z3 * cr * sp - 2 * Z2 * sr * sp;
                d2J_dX2(3, 4) += -2 * (Z2 * cr - Z3 * sr) * (X3 * sp - Z6 * sp - X1 * cp * cy + Z4 * cp * cy - X2 * cp * sy + Z5 * cp * sy);
                d2J_dX2(4, 4) += (Z1 * cp + Z3 * cr * sp + Z2 * sr * sp) * (2 * Z1 * cp + 2 * Z3 * cr * sp + 2 * Z2 * sr * sp) -
                  (2 * Z3 * cr * cp - 2 * Z1 * sp + 2 * Z2 * cp * sr) *(X3 - Z6 - Z1 * sp + Z3 * cr * cp + Z2 * cp * sr) +
                  2 * cy * cy * pow((Z3 * cr * cp - Z1 * sp + Z2 * cp * sr), 2) +2 * sy * sy *
                    pow((Z3 * cr * cp - Z1 * sp + Z2 * cp * sr), 2) - 2 * cy * (Z1 * cp + Z3 * cr * sp + Z2 * sr * sp) *
                    (X1 - Z4 + Z1 * cp * cy - Z2 * cr * sy + Z3 * sr * sy + Z2 * cy * sr * sp + Z3 * cr * cy * sp) -
                  2 * sy * (Z1 * cp + Z3 * cr * sp + Z2 * sr * sp) * (X2 - Z5 + Z2 * cr * cy + Z1 * cp * sy - Z3 * cy * sr +
                     Z3 * cr * sp * sy + Z2 * sr * sp * sy);

                d2J_dX2(0, 5) += 2 * Z3 * (cy * sr - cr * sp * sy) - 2 * Z2 * (cr * cy + sr * sp * sy) - 2 * Z1 * cp * sy;
                d2J_dX2(1, 5) += 2 * Z3 * (sr * sy + cr * cy * sp) - 2 * Z2 * (cr * sy - cy * sr * sp) + 2 * Z1 * cp * cy;
                // d2J_dX2(2,5) += 0;  This quantity is zero
                d2J_dX2(3, 5) +=
                  2 * X1 * Z3 * cr * cy - 2 * Z3 * Z4 * cr * cy +
                  2 * X1 * Z2 * cy * sr + 2 * X2 * Z3 * cr * sy -
                  2 * Z2 * Z4 * cy * sr - 2 * Z3 * Z5 * cr * sy +
                  2 * X2 * Z2 * sr * sy - 2 * Z2 * Z5 * sr * sy +
                  2 * X2 * Z2 * cr * cy * sp - 2 * Z2 * Z5 * cr * cy * sp -
                  2 * X1 * Z2 * cr * sp * sy - 2 * X2 * Z3 * cy * sr * sp +
                  2 * Z2 * Z4 * cr * sp * sy + 2 * Z3 * Z5 * cy * sr * sp +
                  2 * X1 * Z3 * sr * sp * sy - 2 * Z3 * Z4 * sr * sp * sy;
                d2J_dX2(4, 5) += 2 * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr) * (X2 * cy - Z5 * cy - X1 * sy + Z4 * sy);
                d2J_dX2(5, 5) +=
                  2 * Z1 * Z4 * cp * cy - 2 * X2 * Z2 * cr * cy -
                  2 * X1 * Z1 * cp * cy + 2 * Z2 * Z5 * cr * cy +
                  2 * X1 * Z2 * cr * sy - 2 * X2 * Z1 * cp * sy +
                  2 * X2 * Z3 * cy * sr - 2 * Z2 * Z4 * cr * sy +
                  2 * Z1 * Z5 * cp * sy - 2 * Z3 * Z5 * cy * sr -
                  2 * X1 * Z3 * sr * sy + 2 * Z3 * Z4 * sr * sy -
                  2 * X1 * Z3 * cr * cy * sp + 2 * Z3 * Z4 * cr * cy * sp -
                  2 * X1 * Z2 * cy * sr * sp - 2 * X2 * Z3 * cr * sp * sy +
                  2 * Z2 * Z4 * cy * sr * sp + 2 * Z3 * Z5 * cr * sp * sy -
                  2 * X2 * Z2 * sr * sp * sy + 2 * Z2 * Z5 * sr * sp * sy;

                // d2J_dZdX
                // Instead of Filling out this quantity directly,
                // d2J_dZdX*cov(z)*d2J_dZdX' will be calculated for the
                // current correspondence. This is then added elementwise to a
                // running total matrix. This is because
                // the number of columns d2J_dZdX grows linearly with the number
                // of correspondences. This approach can be
                // done because each measurement is assumed independent
                d2J_dZdX(0, 0) = 2 * cp * cy;
                d2J_dZdX(1, 0) = 2 * cy * sr * sp - 2 * cr * sy;
                d2J_dZdX(2, 0) = 2 * sr * sy + 2 * cr * cy * sp;
                // d2J_dZdX(3,0) = -2;
                // d2J_dZdX(4,0) = 0;
                // d2J_dZdX(5,0) = 0;

                d2J_dZdX(0, 1) = 2 * cp * sy;
                d2J_dZdX(1, 1) = 2 * cr * cy + 2 * sr * sp * sy;
                d2J_dZdX(2, 1) = 2 * cr * sp * sy - 2 * cy * sr;
                // d2J_dZdX(3,1) = 0;
                // d2J_dZdX(4,1) = -2;
                // d2J_dZdX(5,1) = 0;

                d2J_dZdX(0, 2) = -2 * sp;
                d2J_dZdX(1, 2) = 2 * cp * sr;
                d2J_dZdX(2, 2) = 2 * cr * cp;
                // d2J_dZdX(3,2) = 0;
                // d2J_dZdX(4,2) = 0;
                // d2J_dZdX(5,2) = -2;

                // d2J_dZdX(0,3) = 0;
                d2J_dZdX(1, 3) = 2 * X3 * cr * cp - 2 * Z6 * cr * cp -
                                 2 * X2 * cy * sr + 2 * Z5 * cy * sr +
                                 2 * X1 * sr * sy - 2 * Z4 * sr * sy +
                                 2 * X2 * cr * sp * sy - 2 * Z5 * cr * sp * sy +
                                 2 * X1 * cr * cy * sp - 2 * Z4 * cr * cy * sp;
                d2J_dZdX(2, 3) = 2 * Z5 * cr * cy - 2 * X2 * cr * cy +
                                 2 * X1 * cr * sy - 2 * X3 * cp * sr -
                                 2 * Z4 * cr * sy + 2 * Z6 * cp * sr -
                                 2 * X1 * cy * sr * sp + 2 * Z4 * cy * sr * sp -
                                 2 * X2 * sr * sp * sy + 2 * Z5 * sr * sp * sy;
                d2J_dZdX(3, 3) = -2 * Z2 * (sr * sy + cr * cy * sp) -
                                 2 * Z3 * (cr * sy - cy * sr * sp);
                d2J_dZdX(4, 3) = 2 * Z2 * (cy * sr - cr * sp * sy) +
                                 2 * Z3 * (cr * cy + sr * sp * sy);
                d2J_dZdX(5, 3) = -2 * cp * (Z2 * cr - Z3 * sr);

                d2J_dZdX(0, 4) = 2 * Z6 * cp - 2 * X3 * cp - 2 * X1 * cy * sp +
                                 2 * Z4 * cy * sp - 2 * X2 * sp * sy +
                                 2 * Z5 * sp * sy;
                d2J_dZdX(1, 4) = -2 * sr * (X3 * sp - Z6 * sp - X1 * cp * cy + Z4 * cp * cy -
                             X2 * cp * sy + Z5 * cp * sy);
                d2J_dZdX(2, 4) = -2 * cr * (X3 * sp - Z6 * sp - X1 * cp * cy + Z4 * cp * cy -
                             X2 * cp * sy + Z5 * cp * sy);
                d2J_dZdX(3, 4) = -2 * cy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dZdX(4, 4) = -2 * sy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dZdX(5, 4) = 2 * Z1 * cp + 2 * Z3 * cr * sp + 2 * Z2 * sr * sp;

                d2J_dZdX(0, 5) = 2 * cp * (X2 * cy - Z5 * cy - X1 * sy + Z4 * sy);
                d2J_dZdX(1, 5) = 2 * Z4 * cr * cy - 2 * X1 * cr * cy -
                                 2 * X2 * cr * sy + 2 * Z5 * cr * sy +
                                 2 * X2 * cy * sr * sp - 2 * Z5 * cy * sr * sp -
                                 2 * X1 * sr * sp * sy + 2 * Z4 * sr * sp * sy;
                d2J_dZdX(2, 5) = 2 * X1 * cy * sr - 2 * Z4 * cy * sr +
                                 2 * X2 * sr * sy - 2 * Z5 * sr * sy -
                                 2 * X1 * cr * sp * sy + 2 * Z4 * cr * sp * sy +
                                 2 * X2 * cr * cy * sp - 2 * Z5 * cr * cy * sp;
                d2J_dZdX(3, 5) = 2 * Z2 * (cr * cy + sr * sp * sy) -
                                 2 * Z3 * (cy * sr - cr * sp * sy) +
                                 2 * Z1 * cp * sy;
                d2J_dZdX(4, 5) = 2 * Z2 * (cr * sy - cy * sr * sp) -
                                 2 * Z3 * (sr * sy + cr * cy * sp) -
                                 2 * Z1 * cp * cy;
                // d2J_dZdX(5,5) = 0;

                middle.noalias() += d2J_dZdX * cov_Z * (d2J_dZdX.transpose());

                // clang-format on
            }
        }
        MatX inverse =
          (d2J_dX2.selfadjointView<Eigen::Upper>()).toDenseMatrix().inverse();
        this->information = (inverse * middle * inverse).inverse();
    }
}

}  // namespace wave
