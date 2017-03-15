// Tests for the correlative scan matcher wrapper
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include "wave/matching/Correlation.hpp"

TEST(cart_tests, initialization) {
    wave::matching::Correlation_Matcher matcher(0.1f);
}

// Should return identity (or close to) for no perturbation
TEST(cart_tests, NullDisplacement) {
    wave::matching::Correlation_Matcher matcher(0.1f);
    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
    pcl::io::loadPCDFile("testscan.pcd", pcl_point_cloud);

    cartographer::sensor::PointCloud point_cloud;
//    for (const auto& point : pcl_point_cloud) {
//        point_cloud.emplace_back(point.x, point.y, point.z);
//    }
    for (const auto& point :
            {Eigen::Vector3f(-3.f, 2.f, 0.f), Eigen::Vector3f(-4.f, 2.f, 0.f),
             Eigen::Vector3f(-5.f, 2.f, 0.f), Eigen::Vector3f(-6.f, 2.f, 0.f),
             Eigen::Vector3f(-6.f, 3.f, 1.f), Eigen::Vector3f(-6.f, 4.f, 2.f),
             Eigen::Vector3f(-7.f, 3.f, 1.f)}) {
        point_cloud.push_back(point);
    }
    matcher.setup(point_cloud, point_cloud);
    EXPECT_EQ(true, matcher.match());
    Eigen::Affine3d result = matcher.get_result();
    EXPECT_EQ(1, result(0,0));
    EXPECT_EQ(1, result(1,1));
    EXPECT_EQ(1, result(2,2));
    EXPECT_EQ(1, result(3,3));
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

