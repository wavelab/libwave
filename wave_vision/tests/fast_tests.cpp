#include "wave/wave_test.hpp"
#include "wave/vision/fast_detector.hpp"

namespace wave {

#define TEST_CONFIG_GOOD "tests/config/fast_good.yaml"
#define TEST_CONFIG_BAD_THRESHOLD "tests/config/fast_bad_threshold.yaml"
#define TEST_CONFIG_BAD_TYPE_NEG "tests/config/fast_bad_type_neg.yaml"
#define TEST_CONFIG_BAD_TYPE_POS "tests/config/fast_bad_type_pos.yaml"
#define TEST_IMAGE "tests/data/lenna.png"

// Test fixture to load same image data
class FASTTest : public testing::Test {
 protected:
    FASTTest() {}
    virtual ~FASTTest() {
        if (this->detector) {
            delete this->detector;
        }
    }
    virtual void SetUp() {
        this->image = cv::imread(TEST_IMAGE, cv::IMREAD_COLOR);
    }

    void initDetector() {
        this->detector = new FASTDetector(TEST_CONFIG_GOOD);
    }

    cv::Mat image;
    FASTDetector *detector;
};

TEST(FASTTests, GoodInitialization) {
    FASTDetector detector(TEST_CONFIG_GOOD);
    SUCCEED();
}

TEST(FASTTests, BadThresholdInitialization) {
    EXPECT_THROW(FASTDetector detector(TEST_CONFIG_BAD_THRESHOLD),
                 ConfigurationLoadingException);
}

TEST(FASTTests, BadTypeInitializationPos) {
    EXPECT_THROW(FASTDetector detector(TEST_CONFIG_BAD_TYPE_POS),
                 ConfigurationLoadingException);
}

TEST(FASTTests, BadTypeInitializationNeg) {
    EXPECT_THROW(FASTDetector detector(TEST_CONFIG_BAD_TYPE_NEG),
                 ConfigurationLoadingException);
}



}  // end of namespace wave