#include <stdio.h>

#include "slam/vision/camera.hpp"
#include "slam/vision/orb.hpp"


int main(void)
{
    int retval;
    char key_pressed;
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    slam::Camera camera;
    slam::ORB orb;

    // setup
    retval = camera.configure(0, 320, 240);
    if (retval != 0) {
        LOG_ERROR("failed to configure camera!");
        return -1;
    }

    retval = orb.configure();
    if (retval != 0) {
        LOG_ERROR("failed to configure ORB detector!");
        return -1;
    }

    // while (true) {
        // camera and feature detector
        camera.getFrame(image);
        orb.detect(image, keypoints);
        orb.compute(image, keypoints, descriptors);

        // visualize
        // cv::drawKeypoints(
        //     image,
        //     keypoints,
        //     image,
        //     cv::Scalar(0, 0, 255)
        // );
        // cv::imshow("Camera", image);

        printf("keypoints size: %d\n", (int) keypoints.size());
        std::cout << descriptors.size() << std::endl;

    //     // handle keyboard events
    //     key_pressed = cv::waitKey(1);
    //     if (key_pressed == 27) {  // press ESC to stop
    //         return 0;
    //     }
    // }

    return 0;
}
