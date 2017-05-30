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

// TEST(BRISKTests, DefaultConstructorTest) {
//     BRISKDescriptor descriptor;
// }

TEST(BRISKTests, BadRadiusList) {
    std::vector<float> r_list_neg = {-1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    std::vector<float> r_list_empty;
    std::vector<int> n_list = {1, 2, 3, 4, 5};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptorParams neg_rlist{r_list_neg, n_list, d_max, d_min};
    BRISKDescriptorParams empty_rlist{r_list_empty, n_list, d_max, d_min};

    ASSERT_THROW(BRISKDescriptor descriptor(neg_rlist), std::invalid_argument);

    ASSERT_THROW(BRISKDescriptor descriptor(empty_rlist),
                 std::invalid_argument);
}

TEST(BRISKTests, BadNumberList) {
    std::vector<float> r_list = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    std::vector<int> n_list_empty;
    std::vector<int> n_list_neg = {-1, 2, 3, 4, 5};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptorParams neg_nlist{r_list, n_list_neg, d_max, d_min};
    BRISKDescriptorParams empty_nlist{r_list, n_list_empty, d_max, d_min};

    ASSERT_THROW(BRISKDescriptor descriptor(neg_nlist), std::invalid_argument);

    ASSERT_THROW(BRISKDescriptor descriptor(empty_nlist),
                 std::invalid_argument);
}

TEST(BRISKTests, UnequalVectorSize) {
    std::vector<float> r_list = {1.0f, 2.0f};
    std::vector<int> n_list = {1, 2, 3};
    float d_max = 5.85f;
    float d_min = 8.2f;

    BRISKDescriptorParams bad_vec_size{r_list, n_list, d_max, d_min};

    ASSERT_THROW(BRISKDescriptor descriptor(bad_vec_size),
                 std::invalid_argument);
}

TEST(BRISKTests, CheckDistValues) {
    std::vector<float> r_list = {1.0f, 2.0f, 3.0f};
    std::vector<int> n_list = {1, 2, 3};
    float d_max_neg = -5.0f;
    float d_max = 5.0f;
    float d_max_large = 20.0f;
    float d_min = 8.0;
    float d_min_neg = -8.0;
    float d_min_small = 1.0f;

    BRISKDescriptorParams neg_dmax = {r_list, n_list, d_max_neg, d_min};
    BRISKDescriptorParams neg_dmin = {r_list, n_list, d_max, d_min_neg};

    // d_max cannot be larger than d_min
    BRISKDescriptorParams swapped_dists = {
      r_list, n_list, d_max_large, d_min_small};

    ASSERT_THROW(BRISKDescriptor descriptor(neg_dmax), std::invalid_argument);
    ASSERT_THROW(BRISKDescriptor descriptor(neg_dmin), std::invalid_argument);
    ASSERT_THROW(BRISKDescriptor descriptor(swapped_dists),
                 std::invalid_argument);
}

} /** end of namespace wave */