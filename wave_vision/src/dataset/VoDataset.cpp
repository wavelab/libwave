#include <boost/filesystem.hpp>

#include "wave/vision/dataset/VoDataset.hpp"

#include <sys/stat.h>

namespace wave {

void VoDatasetGenerator::configure(const std::string &config_file) {
    ConfigParser parser;
    double fx, fy, cx, cy;

    // load config file
    parser.addParam("camera.image_width", &this->camera.image_width);
    parser.addParam("camera.image_height", &this->camera.image_height);
    parser.addParam("camera.fx", &fx);
    parser.addParam("camera.fy", &fy);
    parser.addParam("camera.cx", &cx);
    parser.addParam("camera.cy", &cy);
    parser.addParam("camera.hz", &this->camera.hz);
    parser.addParam("landmarks.nb_landmarks", &this->nb_landmarks);
    parser.addParam("landmarks.x.min", &this->landmark_x_bounds(0));
    parser.addParam("landmarks.x.max", &this->landmark_x_bounds(1));
    parser.addParam("landmarks.y.min", &this->landmark_y_bounds(0));
    parser.addParam("landmarks.y.max", &this->landmark_y_bounds(1));
    parser.addParam("landmarks.z.min", &this->landmark_z_bounds(0));
    parser.addParam("landmarks.z.max", &this->landmark_z_bounds(1));
    if (parser.load(config_file) != 0) {
        throw std::runtime_error("Failed to load " + config_file);
    }

    // set camera matrix
    // clang-format off
  this->camera.K << fx, 0.0, cx,
                    0.0, fy, cy,
                    0.0, 0.0, 1.0;
    // clang-format on
}

LandmarkMap VoDatasetGenerator::generateLandmarks() {
    LandmarkMap landmarks;

    // generate random 3d landmarks
    for (int i = 0; i < this->nb_landmarks; i++) {
        double x =
          randf(this->landmark_x_bounds(0), this->landmark_x_bounds(1));
        double y =
          randf(this->landmark_y_bounds(0), this->landmark_y_bounds(1));
        double z =
          randf(this->landmark_z_bounds(0), this->landmark_z_bounds(1));
        landmarks.emplace(i, Vec3{x, y, z});
    }

    return landmarks;
}

void VoDataset::outputLandmarks(const std::string &output_path) {
    // build landmark matrix
    auto data = MatX{this->landmarks.size(), 4};

    int i = 0;
    for (const auto &landmark : this->landmarks) {
        const auto &landmark_id = landmark.first;
        const auto &point = landmark.second;
        data(i, 0) = landmark_id;
        data.block<1, 3>(i, 1) = point.transpose();
        ++i;
    }

    // output landmarks to file
    auto fmt = Eigen::IOFormat{Eigen::StreamPrecision, Eigen::DontAlignCols};
    std::ofstream out_file{output_path};
    out_file << data.format(fmt) << std::endl;
}

void VoDataset::outputCalibration(const std::string &output_path) {
    std::ofstream calib_file{output_path};
    auto fmt =
      Eigen::IOFormat{Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " "};
    calib_file << this->camera_K.format(fmt);
    calib_file << std::endl;
}

void VoDataset::outputObserved(const std::string &output_dir) {
    // pre-check
    if (this->states.empty()) {
        throw std::runtime_error("outputObserved: no states");
    }

    // output observed features
    int index = 0;
    std::ofstream index_file(output_dir + "/index.dat");

    for (auto state : this->states) {
        // build observed file path
        auto obs_file_name = "observed_" + std::to_string(index) + ".dat";
        auto obs_file_path = output_dir + "/" + obs_file_name;

        // create observed features file
        std::ofstream obs_file(obs_file_path);
        if (!obs_file) {
            throw std::runtime_error("Failed to open " + obs_file_path +
                                     " to output observed features!");
        }

        // output header
        auto fmt = Eigen::IOFormat{
          Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " "};
        obs_file << state.time << std::endl;
        obs_file << state.robot_G_p_GB.format(fmt) << std::endl;
        obs_file << state.robot_q_GB.coeffs().format(fmt) << std::endl;
        obs_file << state.features_observed.size() << std::endl;

        // output observed features
        for (auto feature : state.features_observed) {
            const auto &f_2d = feature.second;  // feature in image frame
            const auto &landmark_id = feature.first;

            obs_file << landmark_id << " " << f_2d.format(fmt) << std::endl;
        }
        obs_file << std::endl;

        // record features observed file path to index
        index_file << obs_file_name << std::endl;
        index++;
    }
    index_file << std::endl;
}

void VoDataset::outputRobotState(const std::string &output_path) {
    // pre-check
    if (this->states.empty()) {
        throw std::runtime_error("outputRobotState: no states");
    }

    // setup
    std::ofstream state_file(output_path);

    // output robot state
    auto fmt =
      Eigen::IOFormat{Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " "};
    for (auto state : this->states) {
        const auto t = state.time;

        state_file << t << " ";
        state_file << state.robot_G_p_GB.format(fmt) << " ";
        state_file << state.robot_q_GB.coeffs().format(fmt) << " ";
        state_file << std::endl;
    }
    state_file << std::endl;
}

void VoDataset::outputToDirectory(const std::string &output_dir) {
    // mkdir calibration directory
    boost::filesystem::create_directories(output_dir);

    // output this synthetic vo dataset
    this->outputLandmarks(output_dir + "/landmarks.dat");
    this->outputCalibration(output_dir + "/calib.dat");
    this->outputRobotState(output_dir + "/state.dat");
    this->outputObserved(output_dir);
}

VoDataset VoDataset::loadFromDirectory(const std::string &input_dir) {
    VoDataset dataset;

    std::ifstream calib_file{input_dir + "/calib.dat"};
    if (!calib_file) {
        throw std::runtime_error("Could not open calib.dat");
    }
    dataset.camera_K = matrixFromStream<3, 3>(calib_file);

    // Landmarks
    std::ifstream landmarks_file{input_dir + "/landmarks.dat"};
    for (LandmarkId id; landmarks_file >> id;) {
        auto landmark_pos = matrixFromStream<3, 1>(landmarks_file);
        dataset.landmarks.emplace(id, landmark_pos);
    }

    // Read the index file to get the file for each state
    std::ifstream index_file{input_dir + "/index.dat"};
    if (!calib_file) {
        throw std::runtime_error("Could not open index.dat");
    }
    for (std::string obs_file_name; index_file >> obs_file_name;) {
        auto obs_file_path = input_dir + "/" + obs_file_name;
        std::ifstream obs_file{obs_file_path};
        if (!obs_file) {
            throw std::runtime_error("Could not open " + obs_file_path);
        }

        VoInstant state;
        obs_file >> state.time;
        state.robot_G_p_GB = matrixFromStream<3, 1>(obs_file);
        state.robot_q_GB = matrixFromStream<4, 1>(obs_file);

        int num_observations;
        obs_file >> num_observations;

        for (auto i = 0; i < num_observations; ++i) {
            LandmarkObservation obs;
            obs_file >> obs.first;
            obs.second = matrixFromStream<2, 1>(obs_file);
            state.features_observed.push_back(obs);
        }

        dataset.states.push_back(state);
    }

    return dataset;
}

VoDataset VoDatasetGenerator::generate() {
    VoDataset dataset;
    // generate random 3D features
    dataset.landmarks = this->generateLandmarks();

    dataset.camera_K = this->camera.K;

    // calculate circle trajectory inputs
    double circle_radius = 0.5;
    double distance = 2 * M_PI * circle_radius;
    double velocity = 1.0;
    double t_end = distance / velocity;
    double angular_velocity = (2 * M_PI) / t_end;
    Vec2 u = Vec2{velocity, angular_velocity};

    // simulate synthetic VO dataset
    double dt = 0.01;
    TwoWheelRobot2DModel robot{Vec3{0.0, 0.0, 0.0}};

    for (int i = 0; i < 300; i++) {
        // update state
        Vec3 pose2d = robot.update(u, dt);

        // convert 2d pose to 3d pose (pose of Body in Global frame)
        auto G_p_GB = Vec3{pose2d.x(), pose2d.y(), 0};
        auto q_GB = Quaternion{Eigen::AngleAxisd{pose2d.z(), Vec3::UnitZ()}};

        auto instant = VoInstant{};

        // Orientation of robot Body frame in Camera frame
        Quaternion q_BC = Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                          Eigen::AngleAxisd(0, Vec3::UnitY()) *
                          Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX());
        Quaternion q_GC = q_GB * q_BC;

        int retval = this->camera.observeLandmarks(
          dt, dataset.landmarks, q_GC, G_p_GB, instant.features_observed);
        if (retval == 0) {
            instant.time = i * dt;
            instant.robot_G_p_GB = G_p_GB;
            instant.robot_q_GB = q_GB;
            dataset.states.push_back(instant);
        }
    }

    return dataset;
}

}  // namespace wave
