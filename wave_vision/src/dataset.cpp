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

int VOTestCamera::checkFeatures(double dt,
                                const MatX &features,
                                const Vec3 &rpy,
                                const Vec3 &t,
                                std::vector<std::pair<Vec2, Vec3>> &observed) {
    // pre-check
    if (this->update(dt) == false) {
        return -1;  // not time to check yet
    }

    // create rotation matrix from roll pitch yaw
    Mat3 R;
    euler2rot(rpy, 123, R);

    // projection matrix
    MatX P;
    projection_matrix(this->K, R, -R * t, P);

    // check which features in 3d are observable from camera
    observed.clear();
    for (int i = 0; i < features.cols(); i++) {
        // convert feature in NWU to EDN coordinate system
        Vec4 f_3d, f_3d_edn;
        f_3d = features.block(0, i, 4, 1);
        f_3d_edn(0) = -f_3d(1);
        f_3d_edn(1) = -f_3d(2);
        f_3d_edn(2) = f_3d(0);
        f_3d_edn(3) = 1.0;

        // project 3D world point to 2D image plane
        Vec3 f_2d;
        f_2d = P * f_3d_edn;

        // check to see if feature is valid and infront of camera
        if (f_2d(2) >= 1.0) {
            // normalize pixels
            f_2d(0) = f_2d(0) / f_2d(2);
            f_2d(1) = f_2d(1) / f_2d(2);
            f_2d(2) = f_2d(2) / f_2d(2);

            // check to see if feature observed is within image plane
            if ((f_2d(0) < this->image_width) && (f_2d(0) > 0)) {
                if ((f_2d(1) < this->image_height) && (f_2d(1) > 0)) {
                    std::pair<Vec2, Vec3> obs;
                    obs = std::make_pair(f_2d.block(0, 0, 2, 1),
                                         f_3d.block(0, 0, 3, 1));
                    observed.push_back(obs);
                }
            }
        }
    }

    return 0;
}


VOTestDataset::VOTestDataset(const std::string &config_file) {
    ConfigParser parser;
    double fx, fy, cx, cy;

    // pre-check
    if (file_exists(config_file) == false) {
        std::string msg = std::string("File does not exist: ") + config_file;
        throw std::runtime_error(msg);
    }

    // load config file
    parser.addParam("camera.image_width", &this->camera.image_width);
    parser.addParam("camera.image_height", &this->camera.image_height);
    parser.addParam("camera.fx", &fx);
    parser.addParam("camera.fy", &fy);
    parser.addParam("camera.cx", &cx);
    parser.addParam("camera.cy", &cy);
    parser.addParam("camera.hz", &this->camera.hz);
    parser.addParam("feature_points.nb_features", &this->nb_features);
    parser.addParam("feature_points.x.min", &this->feature_x_bounds(0));
    parser.addParam("feature_points.x.max", &this->feature_x_bounds(1));
    parser.addParam("feature_points.y.min", &this->feature_y_bounds(0));
    parser.addParam("feature_points.y.max", &this->feature_y_bounds(1));
    parser.addParam("feature_points.z.min", &this->feature_z_bounds(0));
    parser.addParam("feature_points.z.max", &this->feature_z_bounds(1));
    if (parser.load(config_file) != 0) {
        throw std::runtime_error(std::string("Failed to load: ") + config_file);
    }

    // set camera matrix
    // clang-format off
    this->camera.K << fx, 0.0, cx,
                      0.0, fy, cy,
                      0.0, 0.0, 1.0;
    // clang-format on
}

static void prep_header(std::ofstream &output_file) {
    // clang-format off
    output_file << "time_step" << ",";
    output_file << "x" << ",";
    output_file << "y" << ",";
    output_file << "theta" << ",";
    output_file << std::endl;
    // clang-format on
}

static void record_observation(std::ofstream &output_file, const Vec3 &x) {
    output_file << x(0) << ",";
    output_file << x(1) << ",";
    output_file << x(2) << ",";
    output_file << std::endl;
}

void VOTestDataset::generateRandom3DFeatures(MatX &features) {
    Vec4 point;

    // generate random 3d features
    features.resize(4, this->nb_features);
    for (int i = 0; i < this->nb_features; i++) {
        point(0) = randf(this->feature_x_bounds(0), this->feature_x_bounds(1));
        point(1) = randf(this->feature_y_bounds(0), this->feature_y_bounds(1));
        point(2) = randf(this->feature_z_bounds(0), this->feature_z_bounds(1));
        point(3) = 1.0;
        features.block(0, i, 4, 1) = point;
    }
}

void VOTestDataset::record3DFeatures(const std::string &output_path,
                                     const MatX &features) {
    int retval = mat2csv(output_path,
                         features.block(0, 0, 3, features.cols()).transpose());

    if (retval != 0) {
        std::string err_msg =
          std::string("Failed to record 3D features to: ") + output_path;
        throw std::runtime_error(err_msg);
    }
}

void VOTestDataset::recordObservedFeatures(
  double time,
  const Vec3 &x,
  const std::string &output_path,
  std::vector<std::pair<Vec2, Vec3>> &observed) {
    std::ofstream outfile(output_path);
    Vec2 f_2d;
    Vec3 f_3d;

    // open file
    if (outfile.good() != true) {
        std::string err_msg =
          std::string("Failed to open file to record observed features: ") +
          output_path;
        throw std::runtime_error(err_msg);
    }

    // time and number of observed features
    outfile << time << std::endl;
    outfile << observed.size() << std::endl;
    outfile << x(0) << "," << x(1) << "," << x(2) << std::endl;

    // features
    for (auto feature : observed) {
        // feature in image frame
        f_2d = feature.first.transpose();
        outfile << f_2d(0) << "," << f_2d(1) << std::endl;

        // feature in world frame
        f_3d = feature.second.transpose();
        outfile << f_3d(0) << "," << f_3d(1) << "," << f_3d(2) << std::endl;
    }

    // clean up
    outfile.close();
}

void VOTestDataset::generateTestData(const std::string &save_path) {
    // create dataset directory
    int retval = mkdir(save_path.c_str(), ACCESSPERMS);
    if (retval != 0) {
        throw std::runtime_error("Failed to create dataset directory!");
    }

    // setup
    std::ofstream output_file(save_path + "/state.dat");
    std::ofstream index_file(save_path + "/index.dat");
    prep_header(output_file);

    MatX features;
    this->generateRandom3DFeatures(features);
    this->record3DFeatures(save_path + "/features.dat", features);

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
        time += dt;

        // check features
        Vec3 rpy = Vec3{0.0, 0.0, x(2)};
        Vec3 t = Vec3{x(0), x(1), 0.0};
        std::vector<std::pair<Vec2, Vec3>> observed;

        // convert rpy and t from NWU to EDN coordinate system
        Vec3 rpy_edn, t_edn;
        nwu2edn(rpy, rpy_edn);
        nwu2edn(t, t_edn);

        retval = this->camera.checkFeatures(dt, features, rpy_edn, t_edn, observed);
        if (retval == 0) {
            std::ostringstream oss;
            oss.str("");
            oss << save_path + "/observed_" << this->camera.frame << ".dat";
            this->recordObservedFeatures(time, x, oss.str(), observed);
            index_file << oss.str() << std::endl;
        }

        // record state
        record_observation(output_file, x);
    }

    // clean up
    output_file.close();
    index_file.close();
}

}  // wave namespace
