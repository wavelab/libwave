// C++ Headers
#include <algorithm>

// Libwave Headers
#include "wave/wave_test.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/detector/fast_detector.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/descriptor/brisk.yaml";
const auto TEST_IMAGE = "tests/data/lenna.png";

// Test fixture to load same image data
class BRISKTest : public testing::Test {
 protected:
    BRISKTest() {
        initDetector();
        initDescriptor();
        this->image = cv::imread(TEST_IMAGE, cv::IMREAD_COLOR);
        this->keypoints = this->fast.detectFeatures(this->image);
    }
    virtual ~BRISKTest() {}

    void initDescriptor() {
        this->brisk = BRISKDescriptor(TEST_CONFIG);
    }

    void initDetector() {
        this->fast = FASTDetector();
    }

    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    FASTDetector fast;
    BRISKDescriptor brisk;
};

// Checks that correct configuration can be loaded
TEST(BRISKTests, GoodInitialization) {
    EXPECT_NO_THROW(BRISKDescriptor brisk(TEST_CONFIG));
}

// Checks that incorrect configuration path throws an exception
TEST(BRISKTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(BRISKDescriptor brisk(bad_path), std::invalid_argument);
}

TEST(BRISKTests, DefaultConstructorTest) {
    BRISKDescriptor brisk;

    auto check_config = BRISKDescriptorParams{};

    auto config = brisk.getConfiguration();

    for (uint i = 0; i < config.radius_list.size(); i++) {
        ASSERT_NEAR(check_config.radius_list[i], config.radius_list[i], 0.01);
    }
    EXPECT_TRUE(std::equal(check_config.number_list.begin(),
                           check_config.number_list.end(),
                           config.number_list.begin()));
    ASSERT_EQ(check_config.d_max, config.d_max);
    ASSERT_EQ(check_config.d_min, config.d_min);
}

TEST(BRISKTests, CustomParamsConstructorTest) {
    std::vector<float> rlist = {0.0, 2.465, 4.165, 6.29, 9.18};
    std::vector<int> nlist = {1, 10, 14, 15, 20};
    float d_max = 5.85f;
    float d_min = 8.2f;

    // Place defined values into config struct, and create BRISKDescriptor
    // object
    auto input_config = BRISKDescriptorParams(rlist, nlist, d_max, d_min);

    BRISKDescriptor brisk(input_config);

    BRISKDescriptorParams config = brisk.getConfiguration();

    for (uint i = 0; i < config.radius_list.size(); i++) {
        ASSERT_NEAR(input_config.radius_list[i], config.radius_list[i], 0.01);
    }
    EXPECT_TRUE(std::equal(input_config.number_list.begin(),
                           input_config.number_list.end(),
                           config.number_list.begin()));
    ASSERT_EQ(config.d_max, d_max);
    ASSERT_EQ(config.d_min, d_min);
}

TEST(BRISKTests, CustomYamlConstructorTest) {
    std::vector<float> rlist = {0.0, 2.465, 4.165, 6.29, 9.18};
    std::vector<int> nlist = {1, 10, 14, 15, 20};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptor brisk(TEST_CONFIG);

    BRISKDescriptorParams config = brisk.getConfiguration();

    ASSERT_EQ(rlist.size(), config.radius_list.size());
    ASSERT_EQ(nlist.size(), config.number_list.size());

    for (uint i = 0; i < config.radius_list.size(); i++) {
        ASSERT_NEAR(rlist[i], config.radius_list[i], 0.01);
    }
    ASSERT_TRUE(
      std::equal(nlist.begin(), nlist.end(), config.number_list.begin()));
    ASSERT_EQ(d_max, config.d_max);
    ASSERT_EQ(d_min, config.d_min);
}

TEST(BRISKTests, BadRadiusList) {
    std::vector<float> r_list_neg = {-1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    std::vector<float> r_list_empty;
    std::vector<int> n_list = {1, 2, 3, 4, 5};
    float d_max = 5.85f;
    float d_min = 8.2f;

    auto neg_rlist = BRISKDescriptorParams(r_list_neg, n_list, d_max, d_min);
    auto empty_rlist =
      BRISKDescriptorParams(r_list_empty, n_list, d_max, d_min);

    ASSERT_THROW(BRISKDescriptor brisk_rneg(neg_rlist), std::invalid_argument);

    ASSERT_THROW(BRISKDescriptor brisk_rempty(empty_rlist),
                 std::invalid_argument);
}

TEST(BRISKTests, BadNumberList) {
    std::vector<float> r_list = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    std::vector<int> n_list_empty;
    std::vector<int> n_list_neg = {-1, 2, 3, 4, 5};
    float d_max = 5.85f;
    float d_min = 8.2f;

    auto neg_nlist = BRISKDescriptorParams(r_list, n_list_neg, d_max, d_min);
    auto empty_nlist =
      BRISKDescriptorParams(r_list, n_list_empty, d_max, d_min);

    ASSERT_THROW(BRISKDescriptor brisk_nneg(neg_nlist), std::invalid_argument);

    ASSERT_THROW(BRISKDescriptor brisk_nempty(empty_nlist),
                 std::invalid_argument);
}

TEST(BRISKTests, UnequalVectorSize) {
    std::vector<float> r_list = {1.0f, 2.0f};
    std::vector<int> n_list = {1, 2, 3};
    float d_max = 5.85f;
    float d_min = 8.2f;

    auto bad_vec_size = BRISKDescriptorParams(r_list, n_list, d_max, d_min);

    ASSERT_THROW(BRISKDescriptor brisk(bad_vec_size), std::invalid_argument);
}

TEST(BRISKTests, CheckDistValues) {
    std::vector<float> r_list = {1.0f, 2.0f, 3.0f};
    std::vector<int> n_list = {1, 2, 3};
    float d_max_neg = -5.0f;
    float d_max = 5.0f;
    float d_max_large = 20.0f;
    float d_min = 8.0f;
    float d_min_neg = -8.0f;
    float d_min_small = 1.0f;

    auto neg_dmax = BRISKDescriptorParams(r_list, n_list, d_max_neg, d_min);
    auto neg_dmin = BRISKDescriptorParams(r_list, n_list, d_max, d_min_neg);

    // d_max cannot be larger than d_min
    BRISKDescriptorParams swapped_dists = {
      r_list, n_list, d_max_large, d_min_small};

    ASSERT_THROW(BRISKDescriptor brisk(neg_dmax), std::invalid_argument);
    ASSERT_THROW(BRISKDescriptor brisk(neg_dmin), std::invalid_argument);
    ASSERT_THROW(BRISKDescriptor brisk(swapped_dists), std::invalid_argument);
}

TEST_F(BRISKTest, DISABLED_ComputeDescriptors) {
    cv::Mat descriptors;
    cv::Mat image_with_keypoints;

    descriptors = this->brisk.extractDescriptors(this->image, this->keypoints);

    ASSERT_GT(descriptors.total(), 0u);

    // Visual test to verify descriptors are being computed properly
    cv::imshow("Lenna", this->image);
    cv::drawKeypoints(descriptors,
                      this->keypoints,
                      image_with_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("Descriptors", image_with_keypoints);

    cv::waitKey(0);
}

}  // namespace wave
