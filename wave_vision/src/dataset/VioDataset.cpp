#include "wave/vision/dataset/VioDataset.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include "wave/utils/config.hpp"
#include "wave/vision/dataset/VoDataset.hpp"

namespace wave {

// @todo another commit (not in master yet) makes this definition elsewhere
using TimePoint = std::chrono::steady_clock::time_point;

// Helper functions used only in this file
namespace {

}  // namespace

void VioDataset::outputToDirectory(const std::string &output_dir) const {
    // Landmarks - not in kitti. Output using existing vo format
    VoDataset vo;
    vo.landmarks = this->landmarks;
    vo.outputLandmarks(output_dir + "/landmarks.txt");
}

void VioDataset::outputCalibration(const std::string &output_dir) const {
    std::ofstream cam_file{output_dir + "calib_cam_to_cam.txt"};
    auto fmt =
      Eigen::IOFormat{Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " "};
    auto date = boost::posix_time::second_clock::local_time().date();
    boost::posix_time::time_facet

        cam_file
      << "calib_time"
}

static VioDataset loadFromDirectory(const std::string &input_dir) {
    VioDataset dataset;

    std::ifstream calib_file{input_dir + "/calib.dat"};
    std::string throwaway;
    calib_file >> throwaway;

    // Landmarks
    std::ifstream landmarks_file{input_dir + "/landmarks.txt"};
    for (LandmarkId id; landmarks_file >> id;) {
        auto landmark_pos = matrixFromStream<3, 1>(landmarks_file);
        dataset.landmarks.emplace(id, landmark_pos);
    }

    return dataset;
}

}  // namespace wave
