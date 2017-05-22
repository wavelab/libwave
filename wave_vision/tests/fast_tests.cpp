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

    void initDetector() {
        this->detector = new FASTDetector(TEST_CONFIG);
    }

    virtual void SetUp() {
        initDetector();
        this->image = cv::imread(TEST_IMAGE, cv::IMREAD_COLOR);
    }

    cv::Mat image;
    FASTDetector *detector;
    std::vector<cv::KeyPoint> keypoints;
};

//
TEST(FASTTests, GoodInitialization) {
    FASTDetector detector(TEST_CONFIG);
    SUCCEED();
}

TEST(FASTTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(FASTDetector detector(bad_path),
                 ConfigurationLoadingException);
}

TEST(FASTTests, GoodManualConfiguration) {
    FASTDetector detector(TEST_CONFIG);
    int threshold = 10;
    bool nonmax_suppression = true;
    int type = 2;

    int check_threshold;
    bool check_nonmax_suppression;
    int check_type;

    // Configure detector with valid values
    detector.configure(threshold, nonmax_suppression, type);

    // Extract configuration from detector, assert values have been properly set
    detector.getConfiguration(check_threshold,
                              check_nonmax_suppression,
                              check_type);

    ASSERT_EQ(threshold, check_threshold);
    ASSERT_EQ(nonmax_suppression, check_nonmax_suppression);
    ASSERT_EQ(type, check_type);
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

TEST_F(FASTTest, DetectImage) {
    this->keypoints = detector->detectFeatures(this->image);
    //cv::imshow("Lenna", this->image);

    // Draw Keypoints on image and display
    cv::Mat image_with_keypoints;
    cv::drawKeypoints(this->image, this->keypoints, image_with_keypoints,
                      cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    //cv::imshow("Detection Test", image_with_keypoints);

    //cv::waitKey(0);
    SUCCEED();
}

}  // end of namespace wave
