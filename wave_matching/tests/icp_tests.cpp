// Tests for wrapper

#include <iostream>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

#include "wave/matching/ICP.hpp"

bool isApprox(const Eigen::Affine3d &first,
              const Eigen::Affine3d &second,
              double tolerance) {
    for (uint8_t iii = 0; iii < 4; ++iii) {
        for (uint8_t jjj = 0; jjj < 4; ++jjj) {
            tolerance -= std::abs(first(iii, jjj) - second(iii, jjj));
        }
    }
    if (tolerance > 0) {
        return true;
    }
    return false;
}

// Fixture to load same pointcloud all the time
class ICPTest : public testing::Test {
protected:
    ICPTest() {}
    virtual ~ICPTest() {
        if (matcher) {
            delete matcher;
        }
    }
    virtual void SetUp() {
        this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        pcl::io::loadPCDFile("../../wave_matching/tests/testscan.pcd",
                             *(this->ref));
    }
    void setParams(const float res, const Eigen::Affine3d perturb) {
        this->matcher = new wave::ICP_Matcher(res);
        pcl::transformPointCloud(*(this->ref), *(this->target), perturb);
        this->matcher->setup(this->ref, this->target);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref, target;
    wave::ICP_Matcher *matcher;
};

TEST(ICPTests, initialization) {
    wave::ICP_Matcher matcher(0.1f);
}

// Zero displacement without downsampling
TEST_F(ICPTest, fullResNullMatch) {
    Eigen::Affine3d perturb;
    Eigen::Affine3d result;
    bool match_success = false;
    // Setup
    perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0, 0, 0;
    setParams(-1, perturb);
    // test and assert
    match_success = matcher->match();
    result = matcher->getResult();
    EXPECT_TRUE(match_success);
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

// Zero displacement using voxel downsampling
TEST_F(ICPTest, nullDisplacement) {
    Eigen::Affine3d perturb;
    Eigen::Affine3d result;
    bool match_success = false;
    // Setup
    perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0, 0, 0;
    setParams(0.05f, perturb);
    // test and assert
    match_success = matcher->match();
    result = matcher->getResult();
    EXPECT_TRUE(match_success);
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

// Small displacement using voxel downsampling
TEST_F(ICPTest, smallDisplacement) {
    Eigen::Affine3d perturb;
    Eigen::Affine3d result;
    bool match_success = false;
    // Setup
    perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0.2, 0, 0;
    setParams(0.05f, perturb);
    // test and assert
    match_success = matcher->match();
    result = matcher->getResult();
    EXPECT_TRUE(match_success);
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}