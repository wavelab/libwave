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

    // Instantiate cv::BRISK with default values
    float pattern_scale = 1.0f;
    float f = 0.85f * pattern_scale;

    // radiusList contains the radius (in pixels) of each circle in the sampling
    // pattern
    std::vector<float> rlist = {
      f * 0.0f, f * 2.9f, f * 4.9f, f * 7.4f, f * 10.8f};

    std::vector<int> nlist = {1, 10, 14, 15, 20};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptorParams config = brisk.getConfiguration();

    EXPECT_TRUE(
      std::equal(rlist.begin(), rlist.end(), config.radius_list.begin()));
    EXPECT_TRUE(
      std::equal(nlist.begin(), nlist.end(), config.number_list.begin()));
    ASSERT_EQ(config.d_max, d_max);
    ASSERT_EQ(config.d_min, d_min);
}

TEST(BRISKTests, CustomParamsConstructorTest) {
    std::vector<float> rList = {0.0, 2.465, 4.165, 6.29, 9.18};
    std::vector<int> nList = {1, 10, 14, 15, 20};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptorParams input_config{rList, nList, d_max, d_min};

    BRISKDescriptor brisk(input_config);

    BRISKDescriptorParams config = brisk.getConfiguration();

    EXPECT_TRUE(
      std::equal(rList.begin(), rList.end(), config.radius_list.begin()));
    EXPECT_TRUE(
      std::equal(nList.begin(), nList.end(), config.number_list.begin()));
    ASSERT_EQ(config.d_max, d_max);
    ASSERT_EQ(config.d_min, d_min);
}

TEST(BRISKTests, CustomYamlConstructorTest) {
    std::vector<float> rList = {0.0, 2.465, 4.165, 6.29, 9.18};
    std::vector<int> nList = {1, 10, 14, 15, 20};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptor brisk(TEST_CONFIG);

    BRISKDescriptorParams config = brisk.getConfiguration();

    EXPECT_TRUE(
      std::equal(rList.begin(), rList.end(), config.radius_list.begin()));
    EXPECT_TRUE(
      std::equal(nList.begin(), nList.end(), config.number_list.begin()));
    ASSERT_EQ(config.d_max, d_max);
    ASSERT_EQ(config.d_min, d_min);
}

TEST(BRISKTests, BadRadiusList) {
    std::vector<float> r_list_neg = {-1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    std::vector<float> r_list_empty;
    std::vector<int> n_list = {1, 2, 3, 4, 5};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptorParams neg_rlist{r_list_neg, n_list, d_max, d_min};
    BRISKDescriptorParams empty_rlist{r_list_empty, n_list, d_max, d_min};

    ASSERT_THROW(BRISKDescriptor briskr(neg_rlist), std::invalid_argument);

    ASSERT_THROW(BRISKDescriptor brisk(empty_rlist), std::invalid_argument);
}

TEST(BRISKTests, BadNumberList) {
    std::vector<float> r_list = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    std::vector<int> n_list_empty;
    std::vector<int> n_list_neg = {-1, 2, 3, 4, 5};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptorParams neg_nlist{r_list, n_list_neg, d_max, d_min};
    BRISKDescriptorParams empty_nlist{r_list, n_list_empty, d_max, d_min};

    ASSERT_THROW(BRISKDescriptor brisk(neg_nlist), std::invalid_argument);

    ASSERT_THROW(BRISKDescriptor brisk(empty_nlist), std::invalid_argument);
}

TEST(BRISKTests, UnequalVectorSize) {
    std::vector<float> r_list = {1.0f, 2.0f};
    std::vector<int> n_list = {1, 2, 3};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptorParams bad_vec_size{r_list, n_list, d_max, d_min};

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

    BRISKDescriptorParams neg_dmax = {r_list, n_list, d_max_neg, d_min};
    BRISKDescriptorParams neg_dmin = {r_list, n_list, d_max, d_min_neg};

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

}  // end of namespace wave
