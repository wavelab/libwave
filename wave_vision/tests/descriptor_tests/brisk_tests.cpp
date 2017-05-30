// Libwave Headers
#include "wave/wave_test.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"

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
TEST(FASTTests, GoodInitialization) {
    EXPECT_NO_THROW(FASTDetector detector(TEST_CONFIG));
}


}