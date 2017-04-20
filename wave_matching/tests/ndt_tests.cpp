// Tests for wrapper

#include <iostream>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

#include "wave/matching/ndt.hpp"

namespace wave {

// Fixture to load same pointcloud all the time
class NDTTest : public testing::Test {
 protected:
    NDTTest() {}

    virtual ~NDTTest() {
        if (this->matcher) {
            delete this->matcher;
        }
    }

    virtual void SetUp() {
        this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPCDFile("data/testscan.pcd", *(this->ref));
    }

    void initMatcher(const float res, const Eigen::Affine3d perturb) {
        this->matcher = new NDTMatcher(res, "config/ndt.yaml");
        pcl::transformPointCloud(*(this->ref), *(this->target), perturb);
        this->matcher->setup(this->ref, this->target);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ref, target;
    NDTMatcher *matcher;
    const float threshold = 0.12;
};

TEST(NDTTests, initialization) {
    NDTMatcher matcher(1.0f, "config/ndt.yaml");
}

// Zero displacement using resolution from config
TEST_F(NDTTest, fullResNullMatch) {
    Eigen::Affine3d perturb;
    Eigen::Affine3d result;
    bool match_success = false;
    // Setup
    perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0, 0, 0;
    this->initMatcher(-1, perturb);
    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Zero displacement using resolution set by constructor
TEST_F(NDTTest, nullDisplacement) {
    Eigen::Affine3d perturb;
    Eigen::Affine3d result;
    bool match_success = false;
    // Setup
    perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0, 0, 0;
    this->initMatcher(2.5f, perturb);
    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Small displacement using resolution set by constructor
TEST_F(NDTTest, smallDisplacement) {
    Eigen::Affine3d perturb;
    Eigen::Affine3d result;
    bool match_success = false;
    // Setup
    perturb = Eigen::Affine3d::Identity();
    perturb.translation() << 0.2, 0, 0;
    this->initMatcher(2.5f, perturb);
    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

}  // end of namespace wave
