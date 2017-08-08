#include "wave/vision/dataset/VioDataset.hpp"

#include <boost/filesystem.hpp>
#include "wave/utils/config.hpp"
#include "wave/vision/dataset/VoDataset.hpp"

namespace wave {

void VioDataset::outputToDirectory(const std::string &output_dir) const {
    boost::filesystem::create_directories(output_dir);

    // Landmarks - not in kitti. Output using existing vo format
    VoDataset vo;
    vo.landmarks = this->landmarks;
    vo.outputLandmarks(output_dir + "/landmarks.txt");

    this->outputCalibration(output_dir);
}

// Output calib_cam_to_cam.txt
// Most of this is just placeholder values for now, since our dataset class
// doesn't hold any information about distortion or calibration
void VioDataset::outputCalibration(const std::string &output_dir) const {
    std::ofstream cam_file{output_dir + "/calib_cam_to_cam.txt"};
    auto fmt = Eigen::IOFormat{7, Eigen::DontAlignCols, " ", " "};

    // We ignore the calib_time field for now
    cam_file << "calib_time: 01-Jan-2000 12:00:00\n";
    // We ignore the corner_dist field for now (it refers to checkerboard size)
    cam_file << "corner_dist: 0.000000e+00\n";

    // Write only a few ow the parameters for one camera, for now
    // S = size of images
    Vec2 camera_S{this->camera.image_width, this->camera.image_height};
    cam_file << "S_rect_00: " << camera_S.format(fmt) << std::endl;

    // P = projection matrix
    // For the first camera, this is just K with a zero fourth column (for
    // additional cameras, P would include the transformation from the first)
    Eigen::Matrix<double, 3, 4> camera_P;
    camera_P << camera.K, Vec3::Zero();
    cam_file << "P_rect_00: " << camera_P.format(fmt) << std::endl;
    // That's it for now.

    // We have calib cam-to-imu but need to output imu_to_velo and velo_to_cam.
    // For simplicity, choose velo frame equal to imu frame.
    std::ofstream imu_to_velo_file{output_dir + "/calib_imu_to_velo.txt"};
    imu_to_velo_file << "R: " << Mat3::Identity().format(fmt) << std::endl;
    imu_to_velo_file << "T: " << Vec3::Zero().format(fmt) << std::endl;

    // Now velo-to-cam == imu-to-cam
    Mat3 R_CI = this->R_IC.toRotationMatrix().inverse();
    Vec3 C_p_CI = R_CI * -this->I_p_IC;
    std::ofstream velo_to_cam_file{output_dir + "/calib_velo_to_cam.txt"};
    velo_to_cam_file << "R: " << R_CI.format(fmt) << std::endl;
    velo_to_cam_file << "T: " << C_p_CI.format(fmt) << std::endl;
}

VioDataset VioDataset::loadFromDirectory(const std::string &input_dir) {
    VioDataset dataset;

    // The dataset files happen to be valid yaml with the "name: value" format,
    // but the matrix values are not formatted as arrays. To yaml, they are
    // strings.
    //
    // As a quick solution, use the yaml Parser to read strings first. Then
    // re-parse each row as a matrix. We only care about a few fields for now.
    std::string string_S_rect, string_P_rect;

    wave::ConfigParser parser;
    parser.addParam("S_rect_00", &string_S_rect);
    parser.addParam("P_rect_00", &string_P_rect);
    parser.load(input_dir + "/calib_cam_to_cam.txt");

    const auto camera_P = matrixFromString<3, 4>(string_P_rect);
    const auto image_dims = matrixFromString<2, 1>(string_S_rect);
    dataset.camera.K = camera_P.leftCols<3>();
    dataset.camera.image_width = static_cast<int>(image_dims.x());
    dataset.camera.image_height = static_cast<int>(image_dims.y());

    // Now read calib_imu_to_velo and calib_velo_to_cam the same way
    std::string string_R_VI, string_T_VI, string_R_CV, string_T_CV;
    parser = wave::ConfigParser{};
    parser.addParam("R", &string_R_VI);
    parser.addParam("T", &string_T_VI);
    parser.load(input_dir + "/calib_imu_to_velo.txt");

    parser = wave::ConfigParser{};
    parser.addParam("R", &string_R_CV);
    parser.addParam("T", &string_T_CV);
    parser.load(input_dir + "/calib_velo_to_cam.txt");

    const auto R_VI = matrixFromString<3, 3>(string_R_VI);
    const auto R_CV = matrixFromString<3, 3>(string_R_CV);
    const auto V_p_VI = matrixFromString<3, 1>(string_T_VI);
    const auto C_p_CV = matrixFromString<3, 1>(string_T_CV);

    // Now calculate what we really want: calibration imu-to-cam.
    Mat3 R_IC = (R_CV * R_VI).transpose();
    Vec3 C_p_CI = C_p_CV + R_CV * V_p_VI;

    dataset.R_IC.setFromMatrix(R_IC);
    dataset.I_p_IC = R_IC * -C_p_CI;


    // Landmarks
    std::ifstream landmarks_file{input_dir + "/landmarks.txt"};
    for (LandmarkId id; landmarks_file >> id;) {
        auto landmark_pos = matrixFromStream<3, 1>(landmarks_file);
        dataset.landmarks.emplace(id, landmark_pos);
    }

    return dataset;
}

}  // namespace wave
