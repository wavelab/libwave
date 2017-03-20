#include "wave/wave_test.hpp"
#include "wave/utils/math.hpp"


namespace wave {

TEST(Utils_math, median) {
    std::vector<double> v;

    v.push_back(6);
    v.push_back(3);
    v.push_back(4);
    v.push_back(1);
    v.push_back(5);
    v.push_back(8);

    ASSERT_FLOAT_EQ(4.5, median(v));

    v.push_back(9);
    ASSERT_FLOAT_EQ(5.0, median(v));
}

TEST(Utils_math, deg2radAndrad2deg) {
    double d_deg;
    double d_rad;

    d_deg = 10;
    d_rad = deg2rad(d_deg);
    ASSERT_FLOAT_EQ(d_deg, rad2deg(d_rad));
}

TEST(Utils_math, wrapTo180) {
    double retval;

    // normal cases
    retval = wrapTo180(90.0);
    ASSERT_FLOAT_EQ(90.0, retval);

    retval = wrapTo180(180.0);
    ASSERT_FLOAT_EQ(-180.0, retval);

    retval = wrapTo180(270.0);
    ASSERT_FLOAT_EQ(-90.0, retval);

    retval = wrapTo180(360.0);
    ASSERT_FLOAT_EQ(0.0, retval);

    // edge cases
    retval = wrapTo180(-180.0);
    ASSERT_FLOAT_EQ(-180.0, retval);

    retval = wrapTo180(-90.0);
    ASSERT_FLOAT_EQ(-90.0, retval);

    retval = wrapTo180(450.0);
    ASSERT_FLOAT_EQ(90.0, retval);
}

TEST(Utils_math, wrapTo360) {
    double retval;

    // normal cases
    retval = wrapTo360(90.0);
    ASSERT_FLOAT_EQ(90.0, retval);

    retval = wrapTo360(180.0);
    ASSERT_FLOAT_EQ(180.0, retval);

    retval = wrapTo360(270.0);
    ASSERT_FLOAT_EQ(270.0, retval);

    retval = wrapTo360(360.0);
    ASSERT_FLOAT_EQ(0.0, retval);

    retval = wrapTo360(450.0);
    ASSERT_FLOAT_EQ(90.0, retval);

    // edge cases
    retval = wrapTo360(-180.0);
    ASSERT_FLOAT_EQ(180.0, retval);

    retval = wrapTo360(-90.0);
    ASSERT_FLOAT_EQ(270.0, retval);
}

}  // end of wave namespace
