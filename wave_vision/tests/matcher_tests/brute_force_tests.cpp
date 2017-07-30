#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/matcher/brute_force_matcher.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/matcher/brute_force_matcher.yaml";

// Checks that correct configuration can be loaded
TEST(BFTests, GoodInitialization) {
    ASSERT_NO_THROW(BruteForceMatcher bfmatcher);
}

// Checks that incorrect configuration path throws an exception
TEST(BFTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(BFMatcherParams config(bad_path), std::invalid_argument);
}

TEST(BFTests, DefaultConstructorTest) {
    BFMatcherParams check_config(TEST_CONFIG);
    BruteForceMatcher bfmatcher(check_config);

    auto config = bfmatcher.getConfiguration();

    ASSERT_EQ(check_config.norm_type, config.norm_type);
    ASSERT_EQ(check_config.use_knn, config.use_knn);
    ASSERT_EQ(check_config.ratio_threshold, config.ratio_threshold);
    ASSERT_EQ(check_config.distance_threshold, config.distance_threshold);
    ASSERT_EQ(check_config.fm_method, config.fm_method);
}

TEST(BFTests, CustomParamsConstructorTest) {
    int norm_type = cv::NORM_L2;
    bool use_knn = false;
    double ratio_threshold = 0.8;
    int distance_threshold = 5;
    bool auto_remove_outliers = false;
    int fm_method = cv::FM_LMEDS;

    // Place defined values into config struct and create BruteForceMatcher
    BFMatcherParams custom_config(norm_type,
                                  use_knn,
                                  ratio_threshold,
                                  distance_threshold,
                                  auto_remove_outliers,
                                  fm_method);

    BruteForceMatcher bfmatcher(custom_config);

    BFMatcherParams config = bfmatcher.getConfiguration();

    ASSERT_EQ(custom_config.norm_type, config.norm_type);
    ASSERT_EQ(custom_config.use_knn, config.use_knn);
    ASSERT_EQ(custom_config.ratio_threshold, config.ratio_threshold);
    ASSERT_EQ(custom_config.distance_threshold, config.distance_threshold);
    ASSERT_EQ(custom_config.fm_method, config.fm_method);
}

TEST(BFTests, CustomYamlConstructorTest) {
    int norm_type = cv::NORM_HAMMING;
    bool use_knn = true;
    double ratio_threshold = 0.8;
    int distance_threshold = 5;
    bool auto_remove_outliers = true;
    int fm_method = cv::FM_RANSAC;

    BFMatcherParams input_config(TEST_CONFIG);
    BruteForceMatcher bfmatcher(input_config);

    BFMatcherParams config = bfmatcher.getConfiguration();

    ASSERT_EQ(norm_type, config.norm_type);
    ASSERT_EQ(use_knn, config.use_knn);
    ASSERT_EQ(ratio_threshold, config.ratio_threshold);
    ASSERT_EQ(distance_threshold, config.distance_threshold);
    ASSERT_EQ(fm_method, config.fm_method);
}

TEST(BFTests, BadNormType) {
    int bad_norm_type_neg = -1;
    int bad_norm_type_high = 8;
    int bad_norm_type_nd = 3;
    bool use_knn = true;
    double ratio_threshold = 0.8;
    int distance_threshold = 5;
    bool auto_remove_outliers = true;
    int fm_method = cv::FM_RANSAC;

    auto config_neg = BFMatcherParams(bad_norm_type_neg,
                                      use_knn,
                                      ratio_threshold,
                                      distance_threshold,
                                      auto_remove_outliers,
                                      fm_method);
    auto config_high = BFMatcherParams(bad_norm_type_high,
                                       use_knn,
                                       ratio_threshold,
                                       distance_threshold,
                                       auto_remove_outliers,
                                       fm_method);
    auto config_nd = BFMatcherParams(bad_norm_type_nd,
                                     use_knn,
                                     ratio_threshold,
                                     distance_threshold,
                                     auto_remove_outliers,
                                     fm_method);

    ASSERT_THROW(BruteForceMatcher bfmatcher(config_neg),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher(config_high),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher(config_nd), std::invalid_argument);
}

TEST(BFTests, BadRatioTestHeuristic) {
    BFMatcherParams bad_rth_neg;
    BFMatcherParams bad_rth_high;

    bad_rth_neg.ratio_threshold = -0.5;
    bad_rth_high.ratio_threshold = 1.5;

    ASSERT_THROW(BruteForceMatcher bfmatcher_neg(bad_rth_neg),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher_high(bad_rth_high),
                 std::invalid_argument);
}

TEST(BFTests, BadRejectionHeuristic) {
    BFMatcherParams bad_rh_neg;

    bad_rh_neg.distance_threshold = -1;

    ASSERT_THROW(BruteForceMatcher bfmatcher_neg(bad_rh_neg),
                 std::invalid_argument);
}

TEST(BFTests, BadFmMethod) {
    BFMatcherParams low_fm_method;
    BFMatcherParams high_fm_method;
    BFMatcherParams three_fm_method;
    BFMatcherParams mid_fm_method;

    low_fm_method.fm_method = -1;
    high_fm_method.fm_method = 9;
    three_fm_method.fm_method = 3;
    mid_fm_method.fm_method = 5;

    ASSERT_THROW(BruteForceMatcher bfmatcher_low(low_fm_method),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher_high(high_fm_method),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher_three(three_fm_method),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher_mid(mid_fm_method),
                 std::invalid_argument);
}
}  // namespace wave
