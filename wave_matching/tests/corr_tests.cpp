// Tests for the correlative scan matcher wrapper
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <iostream>

#include "wave/matching/correlation.hpp"

// Used to evaluate == for transformations
// Sum of the element-wise absolute differences
// is compared against a threshold
bool isApprox(const Eigen::Affine3d& first,
              const Eigen::Affine3d& second,
              double TOL) {
    for (uint8_t iii = 0; iii < 4; ++iii) {
        for(uint8_t jjj = 0; jjj < 4; ++jjj) {
            TOL -= std::abs(first(iii,jjj) - second(iii,jjj));
        }
    }
    if (TOL > 0) {
        return true;
    }
    return false;
}

TEST(corr_tests, initialization) {
    wave::matching::CorrelationMatcher matcher(0.1f);
}

// Should return identity (or close to) for no perturbation
TEST(corr_tests, NullDisplacement) {
    wave::matching::CorrelationMatcher matcher(0.1f);
    cartographer::sensor::PointCloud point_cloud;
    for (const auto& point :
            {Eigen::Vector3f(-3.f, 2.f, 0.f), Eigen::Vector3f(-4.f, 2.f, 0.f),
             Eigen::Vector3f(-5.f, 2.f, 0.f), Eigen::Vector3f(-6.f, 2.f, 0.f),
             Eigen::Vector3f(-6.f, 3.f, 1.f), Eigen::Vector3f(-6.f, 4.f, 2.f),
             Eigen::Vector3f(-7.f, 3.f, 1.f)}) {
        point_cloud.push_back(point);
    }
    cartographer::sensor::PointCloud dupe = point_cloud;
    Eigen::Affine3d perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0, 0, 0;
    matcher.setup(point_cloud, dupe);
    EXPECT_TRUE(matcher.match());
    Eigen::Affine3d result = matcher.get_result();
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

// This matcher is a grid search, so perturbation must be within search space
TEST(corr_tests, SmallDisplacement) {
    wave::matching::CorrelationMatcher matcher(0.1f);
    cartographer::sensor::PointCloud point_cloud, dupe;
    for (const auto& point :
            {Eigen::Vector3f(-3.f, 2.f, 0.f), Eigen::Vector3f(-4.f, 2.f, 0.f),
             Eigen::Vector3f(-5.f, 2.f, 0.f), Eigen::Vector3f(-6.f, 2.f, 0.f),
             Eigen::Vector3f(-6.f, 3.f, 1.f), Eigen::Vector3f(-6.f, 4.f, 2.f),
             Eigen::Vector3f(-7.f, 3.f, 1.f)}) {
        point_cloud.push_back(point);
    }
    Eigen::Affine3d perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0.2, 0, 0;
    Eigen::Matrix3f mat = perturb.rotation().cast<float> ();
    Eigen::Quaternionf q(mat);
    cartographer::transform::Rigid3f perturb_c(perturb.translation().cast<float>(), q);
    dupe = cartographer::sensor::TransformPointCloud(point_cloud, perturb_c);
    matcher.setup(point_cloud, dupe);
    EXPECT_TRUE(matcher.match());
    Eigen::Affine3d result = matcher.get_result();
    perturb = perturb.inverse();
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

