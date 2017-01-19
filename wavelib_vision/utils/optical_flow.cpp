#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include "slam/vision/vo.hpp"
#include "slam/vision/fast.hpp"
#include "slam/vision/eight_point.hpp"

#define CALIB_FILE "utils/data/calibration.yaml"

struct state
{
    int select_mode;
    int roi_configured;
    cv::Point point_1;
    cv::Point point_2;
};

void state_setup(struct state *ui)
{
    ui->select_mode = 0;
    ui->roi_configured = 0;
    ui->point_1 = cv::Point(0, 0);
    ui->point_2 = cv::Point(0, 0);
}

void mouse_callback(int event, int x, int y, int flags, void *data)
{
    struct state *ui;

    // setup
    ui = (struct state *) data;

    // handle mouse events
    switch (event) {
    case cv::EVENT_LBUTTONDOWN:
        ui->point_1 = cv::Point(x, y);
        ui->point_2 = cv::Point(x, y);
        ui->select_mode = 1;
        ui->roi_configured = 0;
        break;

    case cv::EVENT_MOUSEMOVE:
        if (ui->select_mode) {
            ui->point_2 = cv::Point(x, y);
        }
        break;

    case cv::EVENT_LBUTTONUP:
        ui->point_2 = cv::Point(x, y);
        ui->select_mode = 0;
        ui->roi_configured = 1;
    }
}

void extract_roi(slam::Camera &camera, cv::Mat &mask)
{
    cv::Mat frame;
    cv::Scalar rect_color;
    struct state ui;

    // setup
    state_setup(&ui);
    rect_color = cv::Scalar(0, 0, 255);
    cv::namedWindow("Camera", 1);
    cv::setMouseCallback("Camera", mouse_callback, &ui);

    // loop
    while (true) {
        camera.getFrame(frame);
        if (ui.roi_configured) {
            break;
        } else if (ui.select_mode) {
            cv::rectangle(frame, ui.point_1, ui.point_2, rect_color);
        }

        cv::imshow("Camera", frame);
        cv::waitKey(1);
    }

    // create mask
    std::cout << ui.point_1 << std::endl;
    std::cout << ui.point_2 << std::endl;
    mask(cv::Rect(
        ui.point_1.x,
        ui.point_1.y,
        ui.point_2.x,
        ui.point_2.y
    ));
}

void feature_extraction(
    slam::FastDetector &fast,
    cv::Mat &frame,
    std::vector<cv::Point2f> &points
)
{
    cv::Size win_size;
    cv::Size zero_zone;
    cv::TermCriteria criteria;

    // setup
    win_size = cv::Size(5, 5);
    zero_zone = cv::Size(-1, -1);
    criteria = cv::TermCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
        40,
        0.001
    );

    // detect keypoints
    fast.detect(frame, points);

    // calculate the refined corner locations
    // cv::cornerSubPix(frame, points, win_size, zero_zone, criteria);
}

static void pts2mat(std::vector<cv::Point2f> points, slam::MatX &mat)
{
    cv::Point2f p;

    mat.resize(points.size(), 3);
    for (int i = 0; i < points.size(); i++) {
        p = points[i];
        mat(i, 0) = p.x;
        mat(i, 1) = p.y;
        mat(i, 2) = 1.0;
    }
}

int main(void)
{
    // cv::Mat frame;
    // cv::Mat img_1;
    // cv::Mat img_2;
    // std::vector<cv::Point2f> pts_1;
    // std::vector<cv::Point2f> pts_2;
    // std::vector<float> errors;
    // std::vector<uchar> status;
    // double x, y, z, yaw;
    //
    // std::ofstream output_file;
    //
    // slam::Camera camera;
    // slam::VisualOdometry vo;
    // slam::FastDetector fast;
    //
    // // setup camera
    // camera.configure(0, CALIB_FILE);
    //
    // // setup VO
    // vo.configure(camera.camera_mat);
    // std::cout << "focal length: " << vo.focal_length << std::endl;
    // std::cout << "principle point: " << vo.principle_point << std::endl;
    //
    // // setup feature detector
    // fast.configure(70, true);
    //
    // // setup data file
    // output_file.open("/tmp/odometry.dat");
    // output_file << "i, x, y, z" << std::endl;
    //
    // // loop
    // x = 0.0;
    // y = 0.0;
    // z = 0.0;
    // yaw = 0.0;
    //
    // camera.getFrame(frame);
    // cv::cvtColor(frame, img_1, cv::COLOR_BGR2GRAY);
    // feature_extraction(fast, img_1, pts_1);
    //
    // slam::Mat3 K, E;
    // slam::Vec3 pt1, pt2;
    // slam::MatX pose;
    // slam::MatX epts1;
    // slam::MatX epts2;
    // std::vector<slam::MatX> poses;
    // slam::optimization::EightPoint estimator;
    // estimator.configure(320, 240);
    // K << camera.camera_mat.at<double>(0, 0), camera.camera_mat.at<double>(0, 1), camera.camera_mat.at<double>(0, 2),
    //      camera.camera_mat.at<double>(1, 0), camera.camera_mat.at<double>(1, 1), camera.camera_mat.at<double>(1, 2),
    //      camera.camera_mat.at<double>(2, 0), camera.camera_mat.at<double>(2, 1), camera.camera_mat.at<double>(2, 2);
    // std::cout << K << std::endl;
    //
    // for (int i = 0; i < 100; i++) {
    //     // grab new frame and track features
    //     camera.getFrame(frame);
    //     cv::cvtColor(frame, img_2, cv::COLOR_BGR2GRAY);
    //     vo.featureTracking(img_1, img_2, pts_1, pts_2, errors, status);
    //
    //     pts2mat(pts_1, epts1);
    //     pts2mat(pts_2, epts2);
    //
    //     estimator.estimate(epts1, epts2, K, E);
    //     estimator.obtainPossiblePoses(E, poses);
    //
    //     for (int j = 0; j < errors.size(); j++) {
    //         std::cout << errors[j] << std::endl;
    //     }
    //
    //     pt1 = epts1.block(0, 0, 1, 3).transpose();
    //     pt2 = epts2.block(0, 0, 1, 3).transpose();
    //
    //     estimator.obtainPose(pt1, pt2, K, K, poses, pose);
    //
    //     x += pose(0, 3);
    //     y += pose(1, 3);
    //     z += pose(2, 3);
    //
    //     output_file << i << ", ";
    //     output_file << x << ", ";
    //     output_file << y << ", ";
    //     output_file << z << ", ";
    //     output_file << yaw;
    //     output_file << std::endl;
    //
    //     // measure and record pose estimation
    //     // if (vo.measure(pts_1, pts_2) == 0) {
    //     //     x = vo.t.at<double>(0);
    //     //     y = vo.t.at<double>(1);
    //     //     z = vo.t.at<double>(2);
    //     //     yaw = atan2(vo.R.at<double>(1, 0), vo.R.at<double>(0, 0));
    //     //
    //     //     output_file << i << ", ";
    //     //     output_file << x << ", ";
    //     //     output_file << y << ", ";
    //     //     output_file << z << ", ";
    //     //     output_file << yaw;
    //     //     output_file << std::endl;
    //     // }
    //
    //     // display optical flow
    //     // vo.drawOpticalFlow(frame, pts_1, pts_2);
    //     cv::imshow("test", frame);
    //     cv::waitKey(1);
    //
    //     // update previous image and keypoints
    //     feature_extraction(fast, img_2, pts_1);
    //     img_2.copyTo(img_1);
    //     pts_1 = pts_2;
    // }
    // output_file.close();

    return 0;
}
