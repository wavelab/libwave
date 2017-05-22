#include "wave/wave_test.hpp"
#include "wave/vision/fast_detector.hpp"

namespace wave {

#define TEST_CONFIG "config/fast.yaml"
#define TEST_IMAGE "data/lenna.png"

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
        this->detector = new FASTDetector(TEST_CONFIG);
    }

    cv::Mat image;
    FASTDetector *detector;
};

TEST(FASTTests, GoodInitialization) {
    FASTDetector detector(TEST_CONFIG);
    SUCCEED();
}

TEST(FASTTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(FASTDetector detector(bad_path),
                 ConfigurationLoadingException);
}

TEST(FASTTests, BadThresholdConfiguration) {
    FASTDetector detector(TEST_CONFIG);
    int bad_threshold = -1;
    bool nonmax_suppression = true;
    int type = 2;

    ASSERT_THROW(detector.configure(bad_threshold, nonmax_suppression, type),
                 InvalidConfigurationException);
}

TEST(FASTTests, BadTypeConfiguration) {
    FASTDetector detector(TEST_CONFIG);
    int threshold = 10;
    bool nonmax_suppression = true;
    int bad_type_neg = -1;
    int bad_type_pos = 4;

    ASSERT_THROW(detector.configure(threshold, nonmax_suppression, bad_type_neg),
                 InvalidConfigurationException);

    ASSERT_THROW(detector.configure(threshold, nonmax_suppression, bad_type_pos),
                 InvalidConfigurationException);
}

}  // end of namespace wave
