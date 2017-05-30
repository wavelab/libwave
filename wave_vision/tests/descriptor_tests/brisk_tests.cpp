// Libwave Headers
#include "wave/wave_test.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"

/** The wave namespace */
namespace wave {
const std::string TEST_CONFIG = "tests/config/descriptor/brisk.yaml";
const std::string TEST_IMAGE = "tests/data/lenna.png";

// Test fixture to load same image data
class BRISKTest : public testing::Test {
 protected:
    BRISKTest() {}
    virtual ~BRISKTest() {}

    void initDetector() {
        this->descriptor = BRISKDescriptor(TEST_CONFIG);
    }

    virtual void SetUp() {
        initDetector();
        this->image = cv::imread(TEST_IMAGE, cv::IMREAD_COLOR);
    }

    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    BRISKDescriptor descriptor;
};

// Checks that correct configuration can be loaded
TEST(BRISKTests, GoodInitialization) {
    EXPECT_NO_THROW(BRISKDescriptor descriptor(TEST_CONFIG));
}

// Checks that incorrect configuration path throws an exception
TEST(BRISKTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(BRISKDescriptor descriptor(bad_path), std::invalid_argument);
}

} /** end of namespace wave */