#include "slam/vision/calibration.hpp"


namespace slam {

Calibration::Calibration(void)
{
    this->configured = false;
    this->state = IDEL;

    this->nb_samples = 0;
    this->nb_max_samples = 10;
    this->save_path = "./";
}

int Calibration::configure(
    std::string save_path,
    Chessboard &chessboard,
    cv::Size image_size,
    int nb_max_samples
)
{
    int retval;

    // setup
    rmtrailslash(save_path);

    // mkdir calibration directory
    retval = mkdir(save_path.c_str(), ACCESSPERMS);
    if (retval != 0) {
        switch (errno) {
        case EACCES:
            LOG_ERROR(ECALIBDIRPERM, save_path.c_str());
            break;
        case ENOTDIR:
            LOG_ERROR(ECALIBNOTDIR, save_path.c_str());
            break;
        case EEXIST:
            LOG_ERROR(ECALIBDIREXIST, save_path.c_str());
            break;
        default:
            LOG_ERROR(ECALIBDIR, save_path.c_str());
            break;
        }
        return -1;
    }

    // initialize variables
    this->image_size = image_size;
    this->chessboard = chessboard;
    this->nb_samples = 0;
    this->nb_max_samples = nb_max_samples;
    this->save_path = save_path;

    return 0;
}

bool Calibration::findChessboardCorners(
    cv::Mat &image,
    std::vector<cv::Point2f> &corners
)
{
    int flags;
    bool corners_found;
    cv::Mat image_gray;

    // setup
    this->state = CAPTURING;
    if (corners.size()) {
        corners.clear();
    }

    // detect chessboard corners
    flags = cv::CALIB_CB_ADAPTIVE_THRESH;
    flags += cv::CALIB_CB_NORMALIZE_IMAGE;
    flags += cv::CALIB_CB_FAST_CHECK;
    corners_found = cv::findChessboardCorners(
        image,
        this->chessboard.board_size,
        corners,
        flags
    );

    // draw detected chessboard corners
    cv::drawChessboardCorners(
        image,
        this->chessboard.board_size,
        corners,
        corners_found
    );

    return corners_found;
}

int Calibration::saveImage(cv::Mat &image, std::vector<cv::Point2f> image_points)
{
    std::string image_path;

    // pre-check
    if ((int) image_points.size() != this->chessboard.nb_corners_total) {
        LOG_INFO("failed to detect complete chessboard!");
        return -1;
    } else if (nb_samples >= nb_max_samples) {
        this->state = READY_TO_CALIBRATE;
        LOG_INFO("max calibration samples captured!");
        return -2;
    }

    // save image
    LOG_INFO("captured image [%d]", this->nb_samples);
    image_path = this->save_path + "/";
    image_path += "sample_" + std::to_string(this->nb_samples) + ".jpg";
    cv::imwrite(image_path, image);

    // record image points
    this->nb_samples++;

    return 0;
}

int Calibration::calibrate(
    std::vector<std::vector<cv::Point2f>> image_points,
    cv::Size image_size
)
{
    bool camera_matrix_ok;
    bool distortion_coefficients_ok;
    std::vector<std::vector<cv::Point3f>> object_points(1);

    // pre-check
    if (this->state != READY_TO_CALIBRATE) {
        LOG_INFO("calibrator is not ready to calibrate!");
        return -2;
    }

    // hard-coding the object points - assuming chessboard is origin by setting
    // chessboard in the x-y plane (where z = 0).
    for (int i = 0; i < chessboard.nb_corners_rows; i++) {
        for (int j = 0; j < chessboard.nb_corners_columns; j++) {
            object_points[0].push_back(cv::Point3f(j, i, 0.0f));
        }
    }
    object_points.resize(image_points.size(), object_points[0]);

    // calibrate camera
    this->reprojection_error = cv::calibrateCamera(
        object_points,
        image_points,
        image_size,
        this->camera_matrix,
        this->distortion_coefficients,
        this->rotation_vectors,
        this->translation_vectors
    );

    // check results
    camera_matrix_ok = cv::checkRange(this->camera_matrix);
    distortion_coefficients_ok = cv::checkRange(this->distortion_coefficients);
    if (camera_matrix_ok && distortion_coefficients_ok) {
        return 0;
    } else {
        return -1;
    }
}

static void recordMatrix(YAML::Emitter &out, cv::Mat &mat)
{
    // begin
    out << YAML::BeginMap;

    // rows
    out << YAML::Key << "rows";
    out << YAML::Value << mat.rows;

    // cols
    out << YAML::Key << "cols";
    out << YAML::Value << mat.cols;

    // data
    out << YAML::Key << "data";
    out << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            out << mat.at<double>(i, j);
        }
    }
    out << YAML::EndSeq;

    // end
    out << YAML::EndMap;
}

int Calibration::saveCalibrationOutputs(void)
{
    std::ofstream yaml_file;

    // setup
    yaml_file.open(this->save_path + "/" + CALIBRATION_FILENAME);
    if (yaml_file.bad()) {
        return -1;
    }

    // begin
    this->yaml_config << YAML::BeginMap;

    // image width
    this->yaml_config << YAML::Key << "image_width";
    this->yaml_config << YAML::Value << this->image_size.width;

    // image height
    this->yaml_config << YAML::Key << "image_height";
    this->yaml_config << YAML::Value << this->image_size.height;

    // camera matrix
    this->yaml_config << YAML::Key << "camera_matrix";
    recordMatrix(this->yaml_config, this->camera_matrix);

    // distortion coefficent
    this->yaml_config << YAML::Key << "distortion_coefficients";
    recordMatrix(this->yaml_config, this->distortion_coefficients);

    // distortion coefficent
    this->yaml_config << YAML::Key << "reprojection_error";
    this->yaml_config << YAML::Value << this->reprojection_error;

    // end
    this->yaml_config << YAML::EndMap;

    // output yaml to file
    yaml_file << this->yaml_config.c_str();
    yaml_file.close();

    return 0;
}

} // end of slam namespace
