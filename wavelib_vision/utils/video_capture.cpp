#include "slam/vision/camera.hpp"


int main(void)
{
    slam::Camera camera;
    cv::Mat image;

    int fps;
    int fourcc;
    int retval;
    std::string file_name;
    cv::Size video_resolution;
    cv::VideoWriter video_writer;

    // setup camera
    retval = camera.configure(0, 640, 480);
    if (retval != 0) {
        LOG_ERROR("failed to configure camera!");
        return -1;
    }

    // setup video writer
    fps = 30;
    fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    file_name = "test_video.avi";
    video_resolution = cv::Size(640, 480);

    video_writer = cv::VideoWriter();
    video_writer.open(file_name, fourcc, fps, video_resolution);
    if (video_writer.isOpened() != true) {
        LOG_ERROR("cannot open video writer!");
        return -1;

    } else {
        LOG_INFO("video writer opened!");

    }

    // loop
    while (true) {
        camera.capture->read(image);
        video_writer.write(image);

        cv::imshow("Video Capture", image);
        char c = cv::waitKey(1);
        if (c == 27) break;  // press ESC to stop
    }

    return 0;
}
