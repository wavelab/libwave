#include <boost/filesystem.hpp>

#include "wave/vision/dataset.hpp"

namespace wave {

bool VOTestCamera::update(double dt) {
    this->dt += dt;

    if (this->dt > (1.0 / this->hz)) {
        this->dt = 0.0;
        this->frame++;
        return true;
    }

    return false;
}

int VOTestCamera::observeLandmarks(double dt,
                                   const LandmarkMap &landmarks,
                                   const Quaternion &q_GC,
                                   const Vec3 &G_p_GC,
                                   std::vector<LandmarkObservation> &observed) {
    // pre-check
    if (this->update(dt) == false) {
        return 1;
    }

    // get rotation matrix from world frame to camera frame
    Mat3 R_GC = q_GC.matrix();

    // check which landmarks in 3d are observable from camera
    observed.clear();
    for (auto landmark : landmarks) {
        const auto &point = landmark.second;

        // project 3D world point to 2D image plane, also checking cheirality
        Vec2 meas;
        auto in_front = pinholeProject(this->K, R_GC, G_p_GC, point, meas);
        if (in_front) {
            // check to see if feature observed is within image plane
            if (meas.x() < this->image_width && meas.x() > 0 &&
                meas.y() < this->image_height && meas.y() > 0) {
                observed.emplace_back(landmark.first, meas);
            }
        }
    }
    return 0;
}

void VOTestDatasetGenerator::configure(const std::string &config_file) {
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

LandmarkMap VOTestDatasetGenerator::generateLandmarks() {
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

void VOTestDataset::outputLandmarks(const std::string &output_path) {
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
    auto out_file = std::ofstream{output_path};
    out_file << data.format(fmt) << std::endl;
}

void VOTestDataset::outputCalibration(const std::string &output_path) {
    auto calib_file = std::ofstream{output_path};
    auto fmt =
      Eigen::IOFormat{Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " "};
    calib_file << this->camera_K.format(fmt);
    calib_file << std::endl;
}

void VOTestDataset::outputObserved(const std::string &output_dir) {
    // pre-check
    if (this->states.empty()) {
        throw std::runtime_error("outputObserved: no states");
    }

    // output observed features
    int index = 0;
    std::ofstream index_file(output_dir + "/index.dat");

    for (auto state : this->states) {
        // build observed file path
        std::ostringstream oss("");
        oss << output_dir + "/observed_" << index << ".dat";
        std::string obs_path = oss.str();

        // create observed features file
        std::ofstream obs_file(obs_path);
        if (!obs_file) {
            throw std::runtime_error("Failed to open " + obs_path +
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
        index_file << oss.str() << std::endl;
        index++;
    }
    index_file << std::endl;
}

void VOTestDataset::outputRobotState(const std::string &output_path) {
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

void VOTestDataset::outputToDirectory(const std::string &output_dir) {
    // mkdir calibration directory
    boost::filesystem::create_directories(output_dir);

    // output this synthetic vo dataset
    this->outputLandmarks(output_dir + "/landmarks.dat");
    this->outputCalibration(output_dir + "/calib.dat");
    this->outputRobotState(output_dir + "/state.dat");
    this->outputObserved(output_dir);
}

VOTestDataset VOTestDataset::loadFromDirectory(const std::string &input_dir) {
    auto dataset = VOTestDataset{};

    auto calib_file = std::ifstream{input_dir + "/calib.dat"};
    dataset.camera_K = matrixFromStream<3, 3>(calib_file);

    // Landmarks
    auto landmarks_file = std::ifstream{input_dir + "/landmarks.dat"};
    for (LandmarkId id; landmarks_file >> id;) {
        auto landmark_pos = matrixFromStream<3, 1>(landmarks_file);
        dataset.landmarks.emplace(id, landmark_pos);
    }

    // Read the index file to get the file for each state
    auto index_file = std::ifstream{input_dir + "/index.dat"};
    for (std::string obs_file_name; index_file >> obs_file_name;) {
        auto obs_file = std::ifstream{obs_file_name};

        VOTestInstant state;
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

VOTestDataset VOTestDatasetGenerator::generate() {
    VOTestDataset dataset;
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

        auto instant = VOTestInstant{};

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
