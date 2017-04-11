#include "wave/vision/dataset.hpp"

namespace wave {

TestCamera::TestCamera(void) {
    this->K = Mat3();
    this->image_width = -1;
    this->image_height = -1;
    this->hz = -1;

    this->dt = 0;
}

bool TestCamera::update(double dt) {
    this->dt += dt;

    if (this->dt > (1.0 / this->hz)) {
        this->dt = 0.0;
        return true;
    }

    return false;
}

TestDataset::TestDataset(void) {
    this->configured = false;

    this->camera = TestCamera();

    this->nb_features = -1;
    this->feature_x_bounds = Vec2();
    this->feature_y_bounds = Vec2();
    this->feature_z_bounds = Vec2();
}

int TestDataset::configure(const std::string config_file) {
    ConfigParser parser;
    double fx, fy, cx, cy;

    // pre-check
    if (file_exists(config_file) == false) {
        LOG_ERROR("File does not exist [%s]!", config_file.c_str());
        return -1;
    }

    // load config file
    // parser.addParam("/config/camera/image_width", &this->camera.image_width);
    // parser.addParam("/config/camera/image_height",
    // &this->camera.image_height);
    // parser.addParam("/config/camera/fx", &fx);
    // parser.addParam("/config/camera/fy", &fy);
    // parser.addParam("/config/camera/cx", &cx);
    // parser.addParam("/config/camera/cy", &cy);
    // parser.addParam("/config/camera/hz", &this->camera.hz);
    // parser.addParam("/config/feature_points/nb_features",
    // &this->nb_features);
    // parser.addParam("/config/feature_points/x/min",
    // &this->feature_x_bounds(0));
    // parser.addParam("/config/feature_points/x/max",
    // &this->feature_x_bounds(1));
    // parser.addParam("/config/feature_points/y/min",
    // &this->feature_y_bounds(0));
    // parser.addParam("/config/feature_points/y/max",
    // &this->feature_y_bounds(1));
    // parser.addParam("/config/feature_points/z/min",
    // &this->feature_z_bounds(0));
    // parser.addParam("/config/feature_points/z/max",
    // &this->feature_z_bounds(1));
    // if (parser.load(config_file) != 0) {
    //   LOG_ERROR("Failed to load [%s]!", config_file.c_str());
    //   return -2;
    // }

    // set camera matrix
    // clang-format off
  this->camera.K << fx, 0.0, cx,
                    0.0, fy, cy,
                    0.0, 0.0, 0.0;
    // clang-format on

    this->configured = true;
    return 0;
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

static void record_observation(std::ofstream &output_file, Vec3 &x) {
    output_file << x(0) << ",";
    output_file << x(1) << ",";
    output_file << x(2) << ",";
    output_file << std::endl;
}

void calculate_circle_angular_velocity(double r, double v, double &w) {
    double dist, time;

    dist = 2 * M_PI * r;
    time = dist / v;
    w = (2 * M_PI) / time;
}

int TestDataset::generateRandom3DFeatures(MatX &features) {
    Vec4 point;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // generate random 3d features
    features.resize(4, this->nb_features);
    for (int i = 0; i < this->nb_features; i++) {
        point(0) = randf(this->feature_x_bounds(0), this->feature_x_bounds(1));
        point(1) = randf(this->feature_y_bounds(0), this->feature_y_bounds(1));
        point(2) = randf(this->feature_z_bounds(0), this->feature_z_bounds(1));
        point(3) = 1.0;
        features.block(0, i, 4, 1) = point;
    }

    return 0;
}

int TestDataset::generateTestData(std::string output_path) {
    Vec3 x, t, rpy, f_img;
    Vec4 f_3d;
    Vec2 u;
    MatX features, P;
    double w, dt, time;
    std::ofstream output_file;
    std::vector<std::pair<Vec3, Vec4>> features_observed;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // setup
    output_file.open(output_path);
    prep_header(output_file);
    calculate_circle_angular_velocity(0.5, 1.0, w);
    this->generateRandom3DFeatures(features);

    // initialize states
    dt = 0.01;
    time = 0.0;
    x << 0.0, 0.0, 0.0;
    u << 1.0, w;

    // for (int i = 0; i < 200; i++) {
    for (int i = 0; i < 20; i++) {
        // update state
        x(0) = x(0) + u(0) * cos(x(2)) * dt;
        x(1) = x(1) + u(0) * sin(x(2)) * dt;
        x(2) = x(2) + u(1) * dt;
        time += dt;

        // project 3d feature to image plane
        if (this->camera.update(dt)) {
            rpy << 0.0, 0.0, x(2);
            t << x(0), x(1), 0.0;
            projection_matrix(this->camera.K, rpy, t, P);

            // check which features in 3d are observable from camera
            features_observed.clear();
            for (int j = 0; j < this->nb_features; j++) {
                f_3d = features.block(0, j, 4, 1);
                f_img = P * f_3d;
                features_observed.push_back(std::make_pair(f_img, f_3d));

                std::cout << f_img.transpose() << std::endl;
                std::cout << f_3d.transpose() << std::endl;
                std::cout << std::endl;
            }
        }

        // record state
        record_observation(output_file, x);
    }

    // clean up
    output_file.close();
    return 0;
}

}  // end of wave namespace
