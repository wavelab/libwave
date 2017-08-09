#include "wave/wave_test.hpp"
#include "wave/vision/descriptor/orb_descriptor.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/descriptor/orb.yaml";

// Checks that the default configuration has no issues
TEST(ORBDescriptorTests, GoodConfig) {
    // Default
    EXPECT_NO_THROW(ORBDescriptorParams config1);

    // Custom params struct (with good values)
    int tuple_size = 2;
    int patch_size = 31;

    EXPECT_NO_THROW(ORBDescriptorParams config2(tuple_size, patch_size));

    // From orb.yaml file, with good values.
    EXPECT_NO_THROW(ORBDescriptorParams config3(TEST_CONFIG));
}

// Checks that an incorrect configuration path throws an exception
TEST(ORBDescriptorTests, BadConfigPath) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(ORBDescriptorParams bad_config(bad_path),
                 std::invalid_argument);
}

TEST(ORBDescriptorTests, ConstructorTest) {
    ORBDescriptorParams config;

    EXPECT_NO_THROW(ORBDescriptor descriptor);
    EXPECT_NO_THROW(ORBDescriptor descriptor1(config));
}

// Check that incorrect parameter values throw exceptions.
TEST(ORBDescriptorTests, BadTupleSize) {
    ORBDescriptorParams config;
    config.tuple_size = 1;

    ASSERT_THROW(ORBDescriptor bad_tuplesize1(config), std::invalid_argument);

    config.tuple_size = 5;
    ASSERT_THROW(ORBDescriptor bad_tuplesize2(config), std::invalid_argument);
}

TEST(ORBDescriptorTests, BadPatchSize) {
    ORBDescriptorParams config;
    config.patch_size = -1;

    ASSERT_THROW(ORBDescriptor bad_patchsize(config), std::invalid_argument);
}

TEST(ORBDescriptorTests, ConfigurationTests) {
    ORBDescriptor descriptor;

    ORBDescriptorParams ref_config;
    ORBDescriptorParams curr_config_1 = descriptor.getConfiguration();

    ASSERT_EQ(curr_config_1.tuple_size, ref_config.tuple_size);
    ASSERT_EQ(curr_config_1.patch_size, ref_config.patch_size);

    ORBDescriptorParams new_config{3, 40};

    ASSERT_NO_THROW(descriptor.configure(new_config));

    descriptor.configure(new_config);
    ORBDescriptorParams curr_config_2 = descriptor.getConfiguration();

    ASSERT_EQ(curr_config_2.tuple_size, new_config.tuple_size);
    ASSERT_EQ(curr_config_2.patch_size, new_config.patch_size);
}

TEST(ORBDescriptorTests, BadNewConfiguration) {
    ORBDescriptor descriptor;
    ORBDescriptorParams bad_config{5, 31};

    ASSERT_THROW(descriptor.configure(bad_config), std::invalid_argument);
}
}  // namespace wave
