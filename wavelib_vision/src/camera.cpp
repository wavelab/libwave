#include "slam/vision/camera.hpp"


namespace slam {

Camera::Camera(void)
{
    this->configured = false;

    this->capture_index = 0;
    this->image_width = 0;
    this->image_height = 0;
    this->capture = NULL;
}

Camera::~Camera(void)
{
    if (this->capture != NULL && this->capture->isOpened()) {
        this->capture->release();
    }
}

int Camera::configure(int capture_index, int image_width, int image_height)
{
    // setup
    this->configured = true;
    this->capture = new cv::VideoCapture(capture_index);
    this->capture_index = capture_index;
    this->image_width = image_width;
    this->image_height = image_height;

    // open camera
    if (this->capture->isOpened() == 0) {
        LOG_ERROR("failed to open webcam!");
        return -1;
    }

    // configure image resolution
    this->capture->set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    this->capture->set(cv::CAP_PROP_FRAME_HEIGHT, image_height);

    return 0;
}

static cv::Mat yamlMatrix(YAML::Node matrix_yaml)
{
    int rows;
    int cols;
    int index;
    double value;

    // load matrix
    rows = matrix_yaml["rows"].as<int>();
    cols = matrix_yaml["cols"].as<int>();
    cv::Mat mat(rows, cols, CV_64F);

    index = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            value = matrix_yaml["data"][index].as<double>();
            mat.at<double>(i, j) = value;
            index++;
        }
    }

    return mat;
}

int Camera::configure(int capture_index, std::string calibration_file)
{
    try {
        YAML::Node yaml;

        // setup
        yaml = YAML::LoadFile(calibration_file);

        this->configured = true;
        this->capture = new cv::VideoCapture(capture_index);
        this->capture_index = capture_index;
        this->image_width = yaml["image_width"].as<int>();
        this->image_height = yaml["image_height"].as<int>();
        this->camera_mat = yamlMatrix(yaml["camera_matrix"]);
        this->distortion_coef = yamlMatrix(yaml["distortion_coefficients"]);

        // open camera
        if (this->capture->isOpened() == 0) {
            LOG_ERROR("failed to open webcam!");
            return -1;
        }

        // configure image resolution
        this->capture->set(cv::CAP_PROP_FRAME_WIDTH, image_width);
        this->capture->set(cv::CAP_PROP_FRAME_HEIGHT, image_height);

    } catch (YAML::ParserException ex) {
        std::cout << ex.what() << std::endl;
        return -2;
    }

    return 0;
}

int Camera::getFrame(cv::Mat &frame)
{
    bool retval;

    retval = this->capture->read(frame);
    if (retval == false) {
        return -1;
    }

    return 0;
}

int Camera::getUndistortFrame(cv::Mat &frame)
{
    bool retval;

    retval = this->capture->read(frame);
    if (retval == false) {
        return -1;
    }

    cv::undistort(frame, frame, this->camera_mat, this->distortion_coef);

    return 0;
}

int Camera::saveFrame(cv::Mat &frame, std::string save_path)
{
    bool retval;

    retval = cv::imwrite(save_path, frame);
    if (retval == false) {
        return -1;
    }

    return 0;
}

void Camera::close(void)
{
    if (this->capture != NULL && this->capture->isOpened()) {
        this->capture->release();
    }
}

} // end of slam namespace
