// Tests for wrapper

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <iostream>

#include "wave/matching/NDT.hpp"

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

TEST(ndt_tests, initialization) {
    wave::matching::NDT_Matcher matcher(0.1f);
}

// NDT is apparently too slow for full res
//TEST(ndt_tests, full_res_null_match) {
//    wave::matching::NDT_Matcher matcher(-1.0);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr dupe(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile("../tests/testscan.pcd", *point_cloud);
//    Eigen::Affine3d perturb = Eigen::Affine3d::Identity();
//    perturb.translation() << 0, 0, 0;
//    pcl::transformPointCloud(*point_cloud, *dupe, perturb);
//    matcher.setup(point_cloud, dupe);
//    EXPECT_TRUE(matcher.match());
//    Eigen::Affine3d result = matcher.get_result();
//    EXPECT_TRUE(isApprox(result, perturb, 0.1));
//}

// Should return identity (or close to) for no perturbation
TEST(ndt_tests, NullDisplacement) {
    wave::matching::NDT_Matcher matcher(0.5);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dupe(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../tests/testscan.pcd", *point_cloud);
    Eigen::Affine3d perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0, 0, 0;
    pcl::transformPointCloud(*point_cloud, *dupe, perturb);
    matcher.setup(point_cloud, dupe);
    EXPECT_TRUE(matcher.match());
    Eigen::Affine3d result = matcher.get_result();
//    std::cerr << result.matrix() << std::endl << perturb.matrix();
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

TEST(ndt_tests, SmallDisplacement) {
    wave::matching::NDT_Matcher matcher(0.5);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dupe(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../tests/testscan.pcd", *point_cloud);
    Eigen::Affine3d perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0.2, 0, 0;
    pcl::transformPointCloud(*point_cloud, *dupe, perturb);
    matcher.setup(point_cloud, dupe);
    EXPECT_TRUE(matcher.match());
    Eigen::Affine3d result = matcher.get_result();
//    std::cerr << result.matrix() << std::endl << perturb.matrix();
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}


int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}