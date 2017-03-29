// Tests for wrapper

#include <iostream>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

#include "wave/matching/icp.hpp"

namespace wave {

bool isApprox(const Eigen::Affine3d &first,
              const Eigen::Affine3d &second,
              double tolerance) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            tolerance -= std::abs(first(i, j) - second(i, j));
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
        if (this->matcher) {
            delete this->matcher;
        }
    }

    virtual void SetUp() {
        this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        pcl::io::loadPCDFile("data/testscan.pcd", *(this->ref));
    }

    void setParams(const float res, const Eigen::Affine3d perturb) {
        this->matcher = new ICPMatcher(res);
        pcl::transformPointCloud(*(this->ref), *(this->target), perturb);
        this->matcher->setup(this->ref, this->target);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ref, target;
    ICPMatcher *matcher;
};

TEST(ICPTests, initialization) {
    ICPMatcher matcher(0.1f);
}

// Zero displacement without downsampling
TEST_F(ICPTest, fullResNullMatch) {
    Eigen::Affine3d perturb;
    Eigen::Affine3d result;
    bool match_success = false;
    // Setup
    perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0, 0, 0;
    this->setParams(-1, perturb);
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
    this->setParams(0.05f, perturb);
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
    this->setParams(0.05f, perturb);
    // test and assert
    match_success = matcher->match();
    result = matcher->getResult();
    EXPECT_TRUE(match_success);
    EXPECT_TRUE(isApprox(result, perturb, 0.1));
}

}  // end of namespace wave