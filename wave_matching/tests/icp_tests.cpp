// Tests for wrapper

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <iostream>

#include "wave/matching/ICP.hpp"

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

// Fixture to load same pointcloud all the time
class ICPTest : public testing::Test {
  protected:
    ICPTest() {}
    virtual ~ICPTest() {}
    virtual void SetUp() {
        ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        pcl::io::loadPCDFile("../../wave_matching/tests/testscan.pcd", *ref);
    }
    void setparams(const float res, const Eigen::Affine3d perturb) {
        matcher = new wave::matching::ICP_Matcher(res);
        pcl::transformPointCloud(*ref, *target, perturb);
        matcher->setup(ref, target);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref, target;
    wave::matching::ICP_Matcher *matcher;
};

TEST(icp_tests, initialization) {
    wave::matching::ICP_Matcher matcher(0.1f);
}

// Zero displacement without downsampling
TEST_F(ICPTest, full_res_null_match) {
    Eigen::Affine3d perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0, 0, 0;
    setparams(-1, perturb);
    EXPECT_TRUE(matcher->match());
    Eigen::Affine3d result = matcher->get_result();
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

// Zero displacement using voxel downsampling
TEST_F(ICPTest, NullDisplacement) {
    Eigen::Affine3d perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0, 0, 0;
    setparams(0.05f, perturb);
    EXPECT_TRUE(matcher->match());
    Eigen::Affine3d result = matcher->get_result();
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

// Small displacement using voxel downsampling
TEST_F(ICPTest, SmallDisplacement) {
    Eigen::Affine3d perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0.2, 0, 0;
    setparams(0.05f, perturb);
    EXPECT_TRUE(matcher->match());
    Eigen::Affine3d result = matcher->get_result();
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}