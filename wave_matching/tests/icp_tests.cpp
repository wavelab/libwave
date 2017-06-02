#include <pcl/io/pcd_io.h>

#include "wave/wave_test.hpp"
#include "wave/matching/icp.hpp"

namespace wave {

const auto TEST_SCAN = "data/testscan.pcd";
const auto TEST_CONFIG = "config/icp.yaml";

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
        this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPCDFile(TEST_SCAN, *(this->ref));
    }

    void initMatcher(const float res, const Affine3 perturb) {
        this->matcher = new ICPMatcher(res, TEST_CONFIG);
        pcl::transformPointCloud(*(this->ref), *(this->target), perturb);
        this->matcher->setup(this->ref, this->target);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ref, target;
    ICPMatcher *matcher;
    const float threshold = 0.1;
};

TEST(ICPTests, initialization) {
    ICPMatcher matcher(0.1f, TEST_CONFIG);
}

// Zero displacement without downsampling
TEST_F(ICPTest, fullResNullMatch) {
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

// Zero displacement using voxel downsampling
TEST_F(ICPTest, nullDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0, 0, 0;
    this->initMatcher(0.05f, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Small displacement using voxel downsampling
TEST_F(ICPTest, smallDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;
    this->initMatcher(0.05f, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

}  // namespace wave
