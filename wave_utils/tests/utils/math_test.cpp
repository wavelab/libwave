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

TEST(Utils_math, euler2quat) {
    float roll;
    float pitch;
    float yaw;
    Vec3 euler;
    Quaternion q;

    // check identity quat is returned
    roll = 0;
    pitch = 0;
    yaw = 0;

    euler << roll, pitch, yaw;
    euler2quat(euler, 321, q);
    ASSERT_FLOAT_EQ(0.0, q.x());
    ASSERT_FLOAT_EQ(0.0, q.y());
    ASSERT_FLOAT_EQ(0.0, q.z());
    ASSERT_FLOAT_EQ(1.0, q.w());

    // check valid quat is returned
    roll = M_PI / 2;
    pitch = M_PI;
    yaw = -M_PI / 2;

    euler << roll, pitch, yaw;
    euler2quat(euler, 321, q);
    ASSERT_FLOAT_EQ(0.5, q.x());
    ASSERT_FLOAT_EQ(0.5, q.y());
    ASSERT_FLOAT_EQ(-0.5, q.z());
    ASSERT_FLOAT_EQ(-0.5, q.w());
}

TEST(Utils_math, sandbox) {
    double roll, pitch, yaw;
    Vec3 euler;
    Quaternion q_nwu, q_ned;

    roll = deg2rad(10.0);
    pitch = deg2rad(20.0);
    yaw = deg2rad(30.0);
    euler << roll, pitch, yaw;

    // NWU
    euler2quat(euler, 321, q_nwu);

    // NWU to NED
    nwu2ned(q_nwu, q_ned);
    quat2euler(q_ned, 321, euler);
    ASSERT_FLOAT_EQ(10, rad2deg(euler(0)));
    ASSERT_FLOAT_EQ(-20, rad2deg(euler(1)));
    ASSERT_FLOAT_EQ(-30, rad2deg(euler(2)));

    // NED to NWU
    ned2nwu(q_ned, q_nwu);
    quat2euler(q_nwu, 321, euler);
    ASSERT_FLOAT_EQ(10, rad2deg(euler(0)));
    ASSERT_FLOAT_EQ(20, rad2deg(euler(1)));
    ASSERT_FLOAT_EQ(30, rad2deg(euler(2)));
}

TEST(Utils_math, euler2rot) {
    double roll;
    double pitch;
    double yaw;
    Vec3 euler;
    Mat3 rot;

    double r01, r02, r03;
    double r11, r12, r13;
    double r21, r22, r23;

    // test roll, pitch, yaw set to 0
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    euler << roll, pitch, yaw;
    euler2rot(euler, 321, rot);

    r01 = 1.0;
    r02 = 0.0;
    r03 = 0.0;

    r11 = 0.0;
    r12 = 1.0;
    r13 = 0.0;

    r21 = 0.0;
    r22 = 0.0;
    r23 = 1.0;

    ASSERT_FLOAT_EQ(r01, rot(0, 0));
    ASSERT_FLOAT_EQ(r02, rot(0, 1));
    ASSERT_FLOAT_EQ(r03, rot(0, 2));

    ASSERT_FLOAT_EQ(r11, rot(1, 0));
    ASSERT_FLOAT_EQ(r12, rot(1, 1));
    ASSERT_FLOAT_EQ(r13, rot(1, 2));

    ASSERT_FLOAT_EQ(r21, rot(2, 0));
    ASSERT_FLOAT_EQ(r22, rot(2, 1));
    ASSERT_FLOAT_EQ(r23, rot(2, 2));
}

TEST(Utils_math, enu2nwu) {
    Vec3 enu, nwu;

    enu << 1.0, 2.0, 3.0;
    enu2nwu(enu, nwu);

    ASSERT_FLOAT_EQ(2.0, nwu(0));
    ASSERT_FLOAT_EQ(-1.0, nwu(1));
    ASSERT_FLOAT_EQ(3.0, nwu(2));
}

TEST(Utils_math, nwu2enu) {
    Vec3 enu, nwu;

    nwu << 1.0, 2.0, 3.0;
    nwu2enu(nwu, enu);

    ASSERT_FLOAT_EQ(-2.0, enu(0));
    ASSERT_FLOAT_EQ(1.0, enu(1));
    ASSERT_FLOAT_EQ(3.0, enu(2));
}

TEST(Utils_math, cf2enu) {
    Vec3 cf, enu;

    cf << 1.0, 2.0, 3.0;
    cf2enu(cf, enu);

    ASSERT_FLOAT_EQ(1.0, enu(0));
    ASSERT_FLOAT_EQ(3.0, enu(1));
    ASSERT_FLOAT_EQ(-2.0, enu(2));
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
