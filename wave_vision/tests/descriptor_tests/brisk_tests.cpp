#include "wave/wave_test.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/descriptor/brisk.yaml";

// Checks that default configuration has no issues
TEST(BRISKTests, GoodConfig) {
    // Default
    EXPECT_NO_THROW(BRISKDescriptorParams config1);

    // Custom params struct (with good values)
    std::vector<float> rlist = {0.0, 2.465, 4.165, 6.29, 9.18};
    std::vector<int> nlist = {1, 10, 14, 15, 20};
    float d_max = 5.85f;
    float d_min = 8.2f;

    EXPECT_NO_THROW(BRISKDescriptorParams config2(rlist, nlist, d_max, d_min));

    // From brisk.yaml file, with good vaklues.
    EXPECT_NO_THROW(BRISKDescriptorParams config3(TEST_CONFIG));
}

// Checks that incorrect configuration path throws an exception
TEST(BRISKTests, BadConfigPath) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(BRISKDescriptorParams config(bad_path), std::invalid_argument);
}

TEST(BRISKTests, ConstructorTest) {
    BRISKDescriptorParams config;

    EXPECT_NO_THROW(BRISKDescriptor descriptor);
    EXPECT_NO_THROW(BRISKDescriptor descriptor1(config));
}

// Check that incorrect parameter values throw exceptions
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

TEST(BRISKTests, ConfigurationTests) {
    BRISKDescriptorParams ref_config;
    BRISKDescriptorParams yaml_config(TEST_CONFIG);

    BRISKDescriptor descriptor1(ref_config);
    BRISKDescriptor descriptor2;
    BRISKDescriptor descriptor3(yaml_config);

    BRISKDescriptorParams curr_config_1 = descriptor1.getConfiguration();
    BRISKDescriptorParams curr_config_2 = descriptor2.getConfiguration();
    BRISKDescriptorParams curr_config_3 = descriptor3.getConfiguration();

    // Confirm values for construction with custom struct
    ASSERT_EQ(ref_config.radius_list.size(), curr_config_1.radius_list.size());
    ASSERT_EQ(ref_config.number_list.size(), curr_config_1.number_list.size());
    for (uint i = 0; i < ref_config.radius_list.size(); i++) {
        ASSERT_NEAR(
          ref_config.radius_list[i], curr_config_1.radius_list[i], 0.01);
    }
    ASSERT_TRUE(std::equal(ref_config.number_list.begin(),
                           ref_config.number_list.end(),
                           curr_config_1.number_list.begin()));
    ASSERT_EQ(ref_config.d_max, curr_config_1.d_max);
    ASSERT_EQ(ref_config.d_min, curr_config_1.d_min);

    // Confirm default construction values
    ASSERT_EQ(ref_config.radius_list.size(), curr_config_2.radius_list.size());
    ASSERT_EQ(ref_config.number_list.size(), curr_config_2.number_list.size());
    for (uint i = 0; i < ref_config.radius_list.size(); i++) {
        ASSERT_NEAR(
          ref_config.radius_list[i], curr_config_2.radius_list[i], 0.01);
    }
    ASSERT_TRUE(std::equal(ref_config.number_list.begin(),
                           ref_config.number_list.end(),
                           curr_config_2.number_list.begin()));
    ASSERT_EQ(ref_config.d_max, curr_config_2.d_max);
    ASSERT_EQ(ref_config.d_min, curr_config_2.d_min);

    // Confirm construction with .yaml file
    ASSERT_EQ(ref_config.radius_list.size(), curr_config_3.radius_list.size());
    ASSERT_EQ(ref_config.number_list.size(), curr_config_3.number_list.size());
    for (uint i = 0; i < ref_config.radius_list.size(); i++) {
        ASSERT_NEAR(
          ref_config.radius_list[i], curr_config_3.radius_list[i], 0.01);
    }
    ASSERT_TRUE(std::equal(ref_config.number_list.begin(),
                           ref_config.number_list.end(),
                           curr_config_3.number_list.begin()));
    ASSERT_EQ(ref_config.d_max, curr_config_3.d_max);
    ASSERT_EQ(ref_config.d_min, curr_config_3.d_min);
}
}  // namespace wave
