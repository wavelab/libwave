#include <gtest/gtest.h>

#include "slam/vision/camera.hpp"

#define CALIB_FILE "tests/data/calibration.yaml"


TEST(Camera, constructor)
{
    slam::Camera camera;

    ASSERT_EQ(false, camera.configured);
    ASSERT_EQ(NULL, camera.capture);
    ASSERT_EQ(0, camera.capture_index);
    ASSERT_EQ(0, camera.image_width);
    ASSERT_EQ(0, camera.image_height);
}

TEST(Camera, configure)
{
    int retval;
    cv::Mat image;
    slam::Camera camera;

    // configure camera with index, image dimensions
    retval = camera.configure(0, 320, 240);
    ASSERT_EQ(0, retval);
    camera.close();

    // configure camera with index, calibration file
    camera.configure(0, CALIB_FILE);
    ASSERT_EQ(0, retval);
    camera.close();
}

TEST(Camera, getFrame)
{
    slam::Camera camera;
    cv::Mat image;

    // configure camera with index, image dimensions
    camera.configure(0, 320, 240);
    for (int i = 0; i < 10; i++) {
        camera.getFrame(image);
        cv::imshow("test", image);
        cv::waitKey(1);
    }
    camera.close();
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
