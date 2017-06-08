// C++ Headers
#include <algorithm>

// Libwave Headers
#include "wave/wave_test.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/matcher/brute_force_matcher.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/matcher/brute_force.yaml";
const auto TEST_IMAGE_1 = "tests/data/image_center.png";
const auto TEST_IMAGE_2 = "tests/data/image_right.png";

// Test fixture to load image data
class BFTest : public testing::Test {
 protected:
    BFTest() {
        initDetector();
        initDescriptor();
        initMatcher();
        this->image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
        this->image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);
        this->keypoints_1 = this->fast.detectFeatures(this->image_1);
        this->keypoints_2 = this->fast.detectFeatures(this->image_2);
        this->descriptors_1 =
          this->brisk.extractDescriptors(this->image_1, this->keypoints_1);
        this->descriptors_2 =
          this->brisk.extractDescriptors(this->image_2, this->keypoints_2);
    }

    virtual ~BFTest() {}

    void initDetector() {
        this->fast = FASTDetector();
    }

    void initDescriptor() {
        this->brisk = BRISKDescriptor();
    }

    void initMatcher() {
        this->bfmatcher = BruteForceMatcher(TEST_CONFIG);
    }

    cv::Mat image_1, image_2;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    FASTDetector fast;
    BRISKDescriptor brisk;
    BruteForceMatcher bfmatcher;

};
}