#include <pcl/io/pcd_io.h>

#include "wave/wave_test.hpp"
#include "wave/matching/ndt.hpp"

namespace wave {

#define TEST_SCAN "data/testscan.pcd"
#define TEST_CONFIG "config/ndt.yaml"

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
        pcl::io::loadPCDFile(TEST_SCAN, *(this->ref));
    }

    void initMatcher(const float res, const Affine3 perturb) {
        this->matcher = new NDTMatcher(res, TEST_CONFIG);
        pcl::transformPointCloud(*(this->ref), *(this->target), perturb);
        this->matcher->setup(this->ref, this->target);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ref, target;
    NDTMatcher *matcher;
    const float threshold = 0.12;
};

TEST(NDTTests, initialization) {
    NDTMatcher matcher(1.0f, TEST_CONFIG);
}

// Zero displacement using resolution from config
TEST_F(NDTTest, fullResNullMatch) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
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
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
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
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;
    this->initMatcher(2.5f, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

}  // end of namespace wave
