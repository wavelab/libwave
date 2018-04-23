#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/matcher/flann_matcher.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/matcher/flann.yaml";

// Checks that default configuration has no issues
TEST(FLANNTests, GoodConfig) {
    // Default
    EXPECT_NO_THROW(FLANNMatcherParams config1);

    // Custom params struct (with good values)
    int flann_method = FLANN::KDTree;
    double ratio_threshold = 0.8;
    int distance_threshold = 5;
    bool remove_outliers = true;
    int fm_method = cv::FM_RANSAC;

    EXPECT_NO_THROW(FLANNMatcherParams config2(
      flann_method, ratio_threshold, remove_outliers, fm_method));

    EXPECT_NO_THROW(FLANNMatcherParams config3(
      flann_method, distance_threshold, remove_outliers, fm_method));

    // From flann.yaml, with good values.
    EXPECT_NO_THROW(FLANNMatcherParams config4(TEST_CONFIG));
}

// Checks that incorrect configuration path throws an exception
TEST(FLANNTests, BadConfigPath) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(FLANNMatcherParams config(bad_path), std::invalid_argument);
}

TEST(FLANNTests, ConstructorTest) {
    FLANNMatcherParams config;

    EXPECT_NO_THROW(FLANNMatcher matcher);
    EXPECT_NO_THROW(FLANNMatcher matcher1(config));

    // Check all methods
    config.flann_method = FLANN::KMeans;
    EXPECT_NO_THROW(FLANNMatcher matcher2(config));

    config.flann_method = FLANN::Composite;
    EXPECT_NO_THROW(FLANNMatcher matcher3(config));

    config.flann_method = FLANN::LSH;
    EXPECT_NO_THROW(FLANNMatcher matcher4(config));
}

// Check that incorrect parameter values throw exceptions.
TEST(FLANNTests, BadFLANNMethod) {
    FLANNMatcherParams config;
    config.flann_method = 0;
    ASSERT_THROW(FLANNMatcher bad_norm1(config), std::invalid_argument);

    config.flann_method = 5;
    ASSERT_THROW(FLANNMatcher bad_norm2(config), std::invalid_argument);
}

TEST(FLANNTests, BadRatioTestHeuristic) {
    FLANNMatcherParams config;
    config.ratio_threshold = -0.5;
    ASSERT_THROW(FLANNMatcher bad_rth1(config), std::invalid_argument);

    config.ratio_threshold = 1.5;
    ASSERT_THROW(FLANNMatcher bad_rth2(config), std::invalid_argument);
}

TEST(FLANNTests, BadRejectionHeuristic) {
    FLANNMatcherParams config;
    config.distance_threshold = -1;
    ASSERT_THROW(FLANNMatcher bad_rh(config), std::invalid_argument);
}

TEST(FLANNTests, BadFmMethod) {
    FLANNMatcherParams config;
    config.fm_method = -1;

    ASSERT_THROW(FLANNMatcher bad_fm1(config), std::invalid_argument);

    config.fm_method = 9;
    ASSERT_THROW(FLANNMatcher bad_fm2(config), std::invalid_argument);

    config.fm_method = 3;
    ASSERT_THROW(FLANNMatcher bad_fm3(config), std::invalid_argument);

    config.fm_method = 5;
    ASSERT_THROW(FLANNMatcher bad_fm4(config), std::invalid_argument);
}

TEST(FLANNTests, ConfigurationTests) {
    FLANNMatcherParams ref_config;
    FLANNMatcherParams yaml_config(TEST_CONFIG);

    FLANNMatcher matcher1(ref_config);
    FLANNMatcher matcher2;
    FLANNMatcher matcher3(yaml_config);

    FLANNMatcherParams curr_config_1 = matcher1.getConfiguration();
    FLANNMatcherParams curr_config_2 = matcher2.getConfiguration();
    FLANNMatcherParams curr_config_3 = matcher3.getConfiguration();

    // Confirm values for construction with custom struct
    ASSERT_EQ(curr_config_1.flann_method, ref_config.flann_method);
    ASSERT_EQ(curr_config_1.use_knn, ref_config.use_knn);
    ASSERT_EQ(curr_config_1.ratio_threshold, ref_config.ratio_threshold);
    ASSERT_EQ(curr_config_1.distance_threshold, ref_config.distance_threshold);
    ASSERT_EQ(curr_config_1.remove_outliers, ref_config.remove_outliers);
    ASSERT_EQ(curr_config_1.fm_method, ref_config.fm_method);

    // Confirm default construction
    ASSERT_EQ(curr_config_2.flann_method, ref_config.flann_method);
    ASSERT_EQ(curr_config_2.use_knn, ref_config.use_knn);
    ASSERT_EQ(curr_config_2.ratio_threshold, ref_config.ratio_threshold);
    ASSERT_EQ(curr_config_2.distance_threshold, ref_config.distance_threshold);
    ASSERT_EQ(curr_config_2.remove_outliers, ref_config.remove_outliers);
    ASSERT_EQ(curr_config_2.fm_method, ref_config.fm_method);

    // Confirm construction with .yaml file
    ASSERT_EQ(curr_config_3.flann_method, ref_config.flann_method);
    ASSERT_EQ(curr_config_3.use_knn, ref_config.use_knn);
    ASSERT_EQ(curr_config_3.ratio_threshold, ref_config.ratio_threshold);
    ASSERT_EQ(curr_config_3.distance_threshold, ref_config.distance_threshold);
    ASSERT_EQ(curr_config_3.remove_outliers, ref_config.remove_outliers);
    ASSERT_EQ(curr_config_3.fm_method, ref_config.fm_method);
}
}  // namespace wave
