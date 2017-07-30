#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/matcher/brute_force_matcher.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/matcher/brute_force.yaml";

// Checks that default configuration has no issues
TEST(BFTests, GoodConfig) {
    // Default
    EXPECT_NO_THROW(BFMatcherParams config1);

    // Custom params struct (with good values)
    int norm_type = cv::NORM_HAMMING;
    double ratio_threshold = 0.8;
    int distance_threshold = 5;
    bool auto_remove_outliers = true;
    int fm_method = cv::FM_RANSAC;

    EXPECT_NO_THROW(BFMatcherParams config2(
      norm_type, ratio_threshold, auto_remove_outliers, fm_method));

    EXPECT_NO_THROW(BFMatcherParams config3(
      norm_type, distance_threshold, auto_remove_outliers, fm_method));

    // From brute_force.yaml, with good values.
    EXPECT_NO_THROW(BFMatcherParams config4(TEST_CONFIG));
}

// Checks that incorrect configuration path throws an exception
TEST(BFTests, BadConfigPath) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(BFMatcherParams config(bad_path), std::invalid_argument);
}

TEST(BFTests, ConstructorTest) {
    BFMatcherParams config;

    EXPECT_NO_THROW(BruteForceMatcher matcher);
    EXPECT_NO_THROW(BruteForceMatcher matcher1(config));
}

// Check that incorrect parameter values throw exceptions.
TEST(BFTests, BadNormType) {
    BFMatcherParams config;
    config.norm_type = -1;
    ASSERT_THROW(BruteForceMatcher bad_norm1(config), std::invalid_argument);

    config.norm_type = 8;
    ASSERT_THROW(BruteForceMatcher bad_norm2(config), std::invalid_argument);

    config.norm_type = 3;
    ASSERT_THROW(BruteForceMatcher bad_norm3(config), std::invalid_argument);
}

TEST(BFTests, BadRatioTestHeuristic) {
    BFMatcherParams config;
    config.ratio_threshold = -0.5;
    ASSERT_THROW(BruteForceMatcher bad_rth1(config), std::invalid_argument);

    config.ratio_threshold = 1.5;
    ASSERT_THROW(BruteForceMatcher bad_rth2(config), std::invalid_argument);
}

TEST(BFTests, BadRejectionHeuristic) {
    BFMatcherParams config;
    config.distance_threshold = -1;
    ASSERT_THROW(BruteForceMatcher bad_rh(config), std::invalid_argument);
}

TEST(BFTests, BadFmMethod) {
    BFMatcherParams config;
    config.fm_method = -1;

    ASSERT_THROW(BruteForceMatcher bad_fm1(config), std::invalid_argument);

    config.fm_method = 9;
    ASSERT_THROW(BruteForceMatcher bad_fm2(config), std::invalid_argument);

    config.fm_method = 3;
    ASSERT_THROW(BruteForceMatcher bad_fm3(config), std::invalid_argument);

    config.fm_method = 5;
    ASSERT_THROW(BruteForceMatcher bad_fm4(config), std::invalid_argument);
}

TEST(BFTests, ConfigurationTests) {
    BFMatcherParams ref_config;
    BFMatcherParams yaml_config(TEST_CONFIG);

    BruteForceMatcher matcher1(ref_config);
    BruteForceMatcher matcher2;
    BruteForceMatcher matcher3(yaml_config);

    BFMatcherParams curr_config_1 = matcher1.getConfiguration();
    BFMatcherParams curr_config_2 = matcher2.getConfiguration();
    BFMatcherParams curr_config_3 = matcher3.getConfiguration();

    // Confirm values for construction with custom struct
    ASSERT_EQ(curr_config_1.norm_type, ref_config.norm_type);
    ASSERT_EQ(curr_config_1.use_knn, ref_config.use_knn);
    ASSERT_EQ(curr_config_1.ratio_threshold, ref_config.ratio_threshold);
    ASSERT_EQ(curr_config_1.distance_threshold, ref_config.distance_threshold);
    ASSERT_EQ(curr_config_1.auto_remove_outliers,
              ref_config.auto_remove_outliers);
    ASSERT_EQ(curr_config_1.fm_method, ref_config.fm_method);

    // Confirm default construction
    ASSERT_EQ(curr_config_2.norm_type, ref_config.norm_type);
    ASSERT_EQ(curr_config_2.use_knn, ref_config.use_knn);
    ASSERT_EQ(curr_config_2.ratio_threshold, ref_config.ratio_threshold);
    ASSERT_EQ(curr_config_2.distance_threshold, ref_config.distance_threshold);
    ASSERT_EQ(curr_config_2.auto_remove_outliers,
              ref_config.auto_remove_outliers);
    ASSERT_EQ(curr_config_2.fm_method, ref_config.fm_method);

    // Confirm construction with .yaml file
    ASSERT_EQ(curr_config_3.norm_type, ref_config.norm_type);
    ASSERT_EQ(curr_config_3.use_knn, ref_config.use_knn);
    ASSERT_EQ(curr_config_3.ratio_threshold, ref_config.ratio_threshold);
    ASSERT_EQ(curr_config_3.distance_threshold, ref_config.distance_threshold);
    ASSERT_EQ(curr_config_3.auto_remove_outliers,
              ref_config.auto_remove_outliers);
    ASSERT_EQ(curr_config_3.fm_method, ref_config.fm_method);
}
}  // namespace wave
