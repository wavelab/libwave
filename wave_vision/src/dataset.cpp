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

int VOTestCamera::checkLandmarks(
  double dt,
  const std::map<Vec3, int, VecComparator> &landmarks,
  const Vec3 &rpy,
  const Vec3 &t,
  std::vector<std::pair<Vec2, int>> &observed) {
    // pre-check
    if (this->update(dt) == false) {
        return 1;
    }

    // create rotation matrix from roll pitch yaw
    Mat3 R;
    euler2rot(rpy, 123, R);

    // projection matrix
    MatX P;
    projection_matrix(this->K, R, -R * t, P);

    // check which landmarks in 3d are observable from camera
    observed.clear();
    for (auto landmark : landmarks) {
        // convert feature in NWU to EDN coordinate system
        Vec3 point = landmark.first;
        Vec4 f_3d_edn{-point(1), -point(2), point(0), 1.0};

        // project 3D world point to 2D image plane
        Vec3 f_2d = P * f_3d_edn;

        // check to see if feature is valid and infront of camera
        if (f_2d(2) >= 1.0) {
            // normalize pixels
            f_2d(0) = f_2d(0) / f_2d(2);
            f_2d(1) = f_2d(1) / f_2d(2);
            f_2d(2) = f_2d(2) / f_2d(2);

            // check to see if feature observed is within image plane
            if ((f_2d(0) < this->image_width) && (f_2d(0) > 0)) {
                if ((f_2d(1) < this->image_height) && (f_2d(1) > 0)) {
                    observed.push_back(
                      {f_2d.block(0, 0, 2, 1), landmark.second});
                }
            }
        }
    }

    return 0;
}


int VOTestDataset::configure(const std::string &config_file) {
    ConfigParser parser;
    double fx, fy, cx, cy;

    // pre-check
    if (file_exists(config_file) == false) {
        LOG_ERROR("File does not exist [%s]!", config_file.c_str());
        return -1;
    }

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
        LOG_ERROR("Failed to load [%s]!", config_file.c_str());
        return -2;
    }

    // set camera matrix
    // clang-format off
  this->camera.K << fx, 0.0, cx,
                    0.0, fy, cy,
                    0.0, 0.0, 1.0;
    // clang-format on

    this->configured = true;
    return 0;
}

int VOTestDataset::generateLandmarks() {
    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // generate random 3d features
    for (int i = 0; i < this->nb_landmarks; i++) {
        double x =
          randf(this->landmark_x_bounds(0), this->landmark_x_bounds(1));
        double y =
          randf(this->landmark_y_bounds(0), this->landmark_y_bounds(1));
        double z =
          randf(this->landmark_z_bounds(0), this->landmark_z_bounds(1));
        this->landmarks.insert({Vec3{x, y, z}, i});
    }

    return 0;
}

int VOTestDataset::outputLandmarks(const std::string &output_path) {
    // pre-check
    if (this->configured == false) {
        return -1;
    } else if ((int) this->landmarks.size() != this->nb_landmarks) {
        return -2;
    }

    // build landmark matrix
    MatX data = MatX::Zero(this->landmarks.size(), 3);
    for (auto const &landmark : this->landmarks) {
        Vec3 point = landmark.first;
        int landmark_id = landmark.second;
        data.block(landmark_id, 0, 1, 3) = point.transpose();
    }

    // output landmarks to file
    if (mat2csv(output_path, data) != 0) {
        return -2;
    }

    return 0;
}

int VOTestDataset::outputObserved(const std::string &output_dir) {
    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (this->features_observed.size() == 0) {
        return -2;
    }

    // output observed features
    int index = 0;
    std::ofstream index_file(output_dir + "/index.dat");

    for (auto observed : this->features_observed) {
        // build observed file path
        std::ostringstream oss("");
        oss << output_dir + "/observed_" << index << ".dat";
        std::string obs_path = oss.str();

        // create observed features file
        std::ofstream obs_file(obs_path);
        if (obs_file.good() != true) {
            LOG_ERROR("Failed to open [%s] to output observed features!",
                      obs_path.c_str());
            return -2;
        }

        // output header
        auto state = this->robot_state[index];
        double time = state.first;
        Vec3 x = state.second;
        // clang-format off
				obs_file << time << std::endl;             // time
				obs_file << x(0) << "," << x(1) << "," << x(2) << std::endl;  // robot pose
				obs_file << observed.size() << std::endl;  // number of features
        // clang-format on

        // output observed features
        for (auto feature : observed) {
            // feature in image frame
            Vec2 f_2d = feature.first.transpose();
            obs_file << f_2d(0) << "," << f_2d(1) << std::endl;

            // feature in world frame
            int landmark_id = feature.second;
            obs_file << landmark_id << std::endl;
        }

        // clean up and record features observed file path to index
        obs_file.close();
        index_file << oss.str() << std::endl;
        index++;
    }

    // clean up
    index_file.close();

    return 0;
}

int VOTestDataset::outputRobotState(const std::string &save_path) {
    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (this->robot_state.size() == 0) {
        return -2;
    }

    // setup
    std::ofstream state_file(save_path);

    // state header
    // clang-format off
		state_file << "time_step" << ",";
		state_file << "x" << ",";
		state_file << "y" << ",";
		state_file << "theta" << ",";
		state_file << std::endl;
    // clang-format on

    // output robot state
    for (auto state : this->robot_state) {
        double t = state.first;
        Vec3 x = state.second;

        state_file << t << ",";
        state_file << x(0) << ",";
        state_file << x(1) << ",";
        state_file << x(2) << ",";
        state_file << std::endl;
    }
    state_file.close();

    return 0;
}

int VOTestDataset::simulateVODataset() {
    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // generate random 3D features
    this->generateLandmarks();

    // calculate circle trajectory inputs
    double circle_radius = 0.5;
    double distance = 2 * M_PI * circle_radius;
    double velocity = 1.0;
    double t_end = distance / velocity;
    double angular_velocity = (2 * M_PI) / t_end;
    Vec2 u = Vec2{velocity, angular_velocity};

    // simulate synthetic VO dataset
    double dt = 0.01;
    double time = 0.0;
    TwoWheelRobot2DModel robot{Vec3{0.0, 0.0, 0.0}};

    for (int i = 0; i < 300; i++) {
        // update state
        Vec3 x = robot.update(u, dt);

        // check features
        Vec3 rpy = Vec3{0.0, 0.0, x(2)};
        Vec3 t = Vec3{x(0), x(1), 0.0};
        std::vector<std::pair<Vec2, int>> observed;

        // convert rpy and t from NWU to EDN coordinate system
        Vec3 rpy_edn, t_edn, x_edn;
        nwu2edn(rpy, rpy_edn);
        nwu2edn(t, t_edn);
        nwu2edn(x, x_edn);

        int retval = this->camera.checkLandmarks(
          dt, this->landmarks, rpy_edn, t_edn, observed);
        if (retval == 0) {
            this->robot_state.push_back({time, x});
            this->features_observed.push_back(observed);
        }

        // update
        time += dt;
    }

    return 0;
}

int VOTestDataset::generateTestData(const std::string &output_dir) {
    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // mkdir calibration directory
    int retval = mkdir(output_dir.c_str(), ACCESSPERMS);
    if (retval != 0) {
        LOG_ERROR("Failed to create dataset directory!");
        return -2;
    }

    // output synthetic vo dataset
    this->simulateVODataset();
    if (this->outputLandmarks(output_dir + "/landmarks.dat") != 0)
        goto error;
    if (this->outputRobotState(output_dir + "/state.dat") != 0)
        goto error;
    if (this->outputObserved(output_dir) != 0)
        goto error;

    return 0;
error:
    return -3;
}

}  // namespace wave
