#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <pointmatcher/PointMatcher.h>

#include "wave/wave_test.hpp"
#include "wave/matching/lpm_icp.hpp"

namespace wave {

const auto TEST_SCAN = "tests/data/testscan.pcd";
const auto TEST_CONFIG = "tests/config/lpm_icp.yaml";


class LPMICPTest : public testing::Test {
protected:
    LPMICPTest() {}

    virtual ~LPMICPTest() {
        if (this->matcher) {
            delete this->matcher;
        }
    }

    virtual void SetUp() {
        this->ref = boost::make_shared<PointMatcher<double>::DataPoints>();
        this->target = boost::make_shared<PointMatcher<double>::DataPoints>();
        *(this->ref) = PointMatcher<double>::DataPoints::load("tests/data/testscan_ascii.pcd");
    }

    void initMatcher(const std::string config_file, const Affine3 perturb) {
        this->matcher = new LPMMatcher(config_file);
        PointMatcher<double>::TransformationParameters T;
        T = perturb.matrix();
        PointMatcher<double>::Transformation* rigidTrans;
        rigidTrans = PointMatcher<double>::get().REG(Transformation).create("RigidTransformation");
        *(this->target) = rigidTrans->compute(*(this->ref), T);
        this->matcher->setup(this->ref, this->target);
    }

    boost::shared_ptr<PointMatcher<double>::DataPoints> ref, target;
    LPMMatcher *matcher;
    const float threshold = 0.1;
};

TEST_F(LPMICPTest, fullResNullMatch) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    perturb = Affine3::Identity();
    perturb.translation() << 0, 0, 0;
    this->initMatcher(TEST_CONFIG, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Zero displacement using voxel downsampling
TEST_F(LPMICPTest, nullDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0, 0, 0;
    this->initMatcher(TEST_CONFIG, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}


// Small displacement using voxel downsampling
TEST_F(LPMICPTest, smallDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;
    this->initMatcher(TEST_CONFIG, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Small information using voxel downsampling
TEST_F(LPMICPTest, smallinfo) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;
    this->initMatcher(TEST_CONFIG, perturb);

    // test and assert
    match_success = matcher->match();
    matcher->estimateInfo();
    auto info = matcher->getInfo();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_GT(info(0, 0), 0);
    EXPECT_LT(diff, this->threshold);
}
} // namespace wave