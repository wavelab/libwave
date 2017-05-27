// Libwave Headers
#include "wave/wave_test.hpp"
#include "wave/vision/detector/fast_detector.hpp"

namespace wave {

const std::string TEST_CONFIG = "tests/config/detector/fast.yaml";
const std::string TEST_IMAGE = "tests/data/lenna.png";

// Test fixture to load same image data
class FASTTest : public testing::Test {
 protected:
    FASTTest() {}
    virtual ~FASTTest() {}

    void initDetector() {
        this->detector = FASTDetector(TEST_CONFIG);
    }

    virtual void SetUp() {
        initDetector();
        this->image = cv::imread(TEST_IMAGE, cv::IMREAD_COLOR);
    }

    cv::Mat image;
    FASTDetector detector;
    std::vector<cv::KeyPoint> keypoints;
};

// Checks that correct configuration can be loaded
TEST(FASTTests, GoodInitialization) {
    EXPECT_NO_THROW(FASTDetector detector(TEST_CONFIG));
}

// Checks that incorrect configuration path throws an exception
TEST(FASTTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(FASTDetector detector(bad_path),
                 std::invalid_argument);
}

// Checks that invalid threshold value throws the proper exception
TEST(FASTTests, BadThresholdConfiguration) {
    FASTDetector detector(TEST_CONFIG);

    FASTParams bad_threshold_config{-1, true, 2};

    ASSERT_THROW(detector.configure(bad_threshold_config),
                 std::invalid_argument);
}

// Checks that invalid type values throw the proper exception
TEST(FASTTests, BadTypeConfiguration) {
    FASTDetector detector(TEST_CONFIG);

    FASTParams bad_type_config_neg{10, true, -1};
    FASTParams bad_type_config_pos{10, true, 4};

    ASSERT_THROW(detector.configure(bad_type_config_neg),
                 std::invalid_argument);

    ASSERT_THROW(detector.configure(bad_type_config_pos),
                 std::invalid_argument);
}

TEST(FASTTests, DefaultConstructorTest) {
    FASTDetector detector;

    FASTParams config;

    config = detector.getConfiguration();

    ASSERT_EQ(config.threshold, 10);
    ASSERT_EQ(config.nonmax_suppression, true);
    ASSERT_EQ(config.type, 2);
}

// Checks that correct configuration values can be set in detector, and also
// read from getConfiguration function.
TEST(FASTTests, GoodPathConfiguration) {
    FASTDetector detector(TEST_CONFIG);

    FASTParams input_config{10, true, 2};
    FASTParams output_config;

    // Configure detector with valid values
    detector.configure(input_config);

    // Extract configuration from detector, assert values have been properly set
    output_config = detector.getConfiguration();

    ASSERT_EQ(output_config.threshold, input_config.threshold);
    ASSERT_EQ(output_config.nonmax_suppression,
              input_config.nonmax_suppression);
    ASSERT_EQ(output_config.type, input_config.type);
}

// Confirms that keypoints can be determined, and image with keypoints can be
// displayed.
TEST_F(FASTTest, DISABLED_DetectImage) {
    this->keypoints = detector.detectFeatures(this->image);
    ASSERT_NE(this->keypoints.size(), 0u);

    cv::Mat extracted_image = this->detector.getImage();
    cv::imshow("Lenna", extracted_image);

    // Visual test, to confirm that images are displayed properly
    // Draw Keypoints on image and display
    cv::Mat image_with_keypoints;
    cv::drawKeypoints(this->image,
                      this->keypoints,
                      image_with_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("Detection Test", image_with_keypoints);

    cv::waitKey(0);
}
}  // end of namespace wave
