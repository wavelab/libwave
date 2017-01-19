#include <gtest/gtest.h>

#include "slam/gimbal/sbgc.hpp"


TEST(SBGC, connectAndDisconnect)
{
    slam::SBGC sbgc("/dev/ttyUSB0");
    ASSERT_EQ(0, sbgc.connect());
    ASSERT_EQ(0, sbgc.disconnect());
}

TEST(SBGC, sendFrame)
{
    int retval;
    slam::SBGCFrame frame;
    slam::SBGC sbgc("/dev/ttyUSB0");

    // setup
    sbgc.connect();

    // turn motors on
    frame.buildFrame(CMD_MOTORS_ON);
    retval = sbgc.sendFrame(frame);
    ASSERT_EQ(0, retval);
    sleep(1);

    // turn motors off
    frame.buildFrame(CMD_MOTORS_OFF);
    retval = sbgc.sendFrame(frame);
    ASSERT_EQ(0, retval);
    sleep(1);
}

TEST(SBGC, readFrame)
{
    int retval;
    slam::SBGCFrame frame;
    slam::SBGC sbgc("/dev/ttyUSB0");

    // setup
    sbgc.connect();

    // read frame
    frame.buildFrame(CMD_BOARD_INFO);
    sbgc.sendFrame(frame);
    retval = sbgc.readFrame(CMD_BOARD_INFO_FRAME_SIZE, frame);

    // assert
    ASSERT_EQ(18, frame.data_size);
    ASSERT_EQ(0, retval);
}

TEST(SBGC, getBoardInfo)
{
    slam::SBGC sbgc("/dev/ttyUSB0");

    // setup
    sbgc.connect();

    // test get board info
    sbgc.getBoardInfo();
    printf("board version: %d\n", sbgc.board_version);
    printf("firmware version: %d\n", sbgc.firmware_version);
    printf("debug mode: %d\n", sbgc.debug_mode);
    printf("board features: %d\n", sbgc.board_features);
    printf("connection flags: %d\n", sbgc.connection_flags);

    ASSERT_EQ(30, sbgc.board_version);
    ASSERT_EQ(2569, sbgc.firmware_version);
}

TEST(SBGC, getRealtimeData)
{
    slam::SBGC sbgc("/dev/ttyUSB0");

    // setup
    sbgc.connect();

    // test get imu data
    for (int i = 0; i < 100; ++i){
        sbgc.getRealtimeData();
        sbgc.data.printData();
        // printf("roll %f \n", sbgc.data.rc_angles(0));
        // printf("pitch %f \n", sbgc.data.rc_angles(1));
        // printf("yaw %f \n", sbgc.data.rc_angles(2));
    }
}

TEST(SBGC, setAngle)
{
    slam::SBGC sbgc("/dev/ttyUSB0");

    ASSERT_EQ(0, sbgc.connect());
    sbgc.on();

    sbgc.setAngle(0, -90, 0);
    sleep(2);
    for (int angle = -95; angle < 20; angle += 3){
        sbgc.setAngle(0, angle, 0);
    }
    sbgc.off();
}

TEST(SBGC, setSpeedAngle)
{
    slam::SBGC sbgc("/dev/ttyUSB0");

    ASSERT_EQ(0, sbgc.connect());
    sbgc.on();

    sbgc.setSpeedAngle(0, 10, 0, 0, -2, 0);
    sleep(3);

    sbgc.off();
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
