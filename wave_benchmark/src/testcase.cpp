#include "wave/benchmark/testcase.hpp"

namespace wave {

bool TestCamera::update(double dt) {
  this->dt += dt;

  if (this->dt > (1.0 / this->hz)) {
    this->dt = 0.0;
    this->frame++;
    return true;
  }

  return false;
}

int TestCamera::checkFeatures(double dt,
                              const MatX &features,
                              const Vec3 &rpy,
                              const Vec3 &t,
                              std::vector<std::pair<Vec2, Vec3>> &observed) {
  Vec3 f_2d, rpy_edn, t_edn;
  std::pair<Vec2, Vec3> obs;
  Vec4 f_3d, f_3d_edn;
  Mat3 R;
  MatX P;

  // pre-check
  if (this->update(dt) == false) {
    return 1;
  }

  // rotation matrix - convert from nwu to edn then to rotation matrix R
  nwu2edn(rpy, rpy_edn);
  euler2rot(rpy_edn, 123, R);

  // translation - convert translation from nwu to edn
  nwu2edn(t, t_edn);

  // projection matrix
  projection_matrix(this->K, R, -R * t_edn, P);

  // check which features in 3d are observable from camera
  observed.clear();
  for (int i = 0; i < features.cols(); i++) {
    // convert feature in NWU to EDN coordinate system
    f_3d = features.block(0, i, 4, 1);
    f_3d_edn(0) = -f_3d(1);
    f_3d_edn(1) = -f_3d(2);
    f_3d_edn(2) = f_3d(0);
    f_3d_edn(3) = 1.0;

    // project 3D world point to 2D image plane
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
          obs =
            std::make_pair(f_2d.block(0, 0, 2, 1), f_3d.block(0, 0, 3, 1));
          observed.push_back(obs);
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
    log_error("File does not exist [%s]!", config_file.c_str());
    return -1;
  }

  // load config file
  parser.addParam("/config/camera/image_width", &this->camera.image_width);
  parser.addParam("/config/camera/image_height", &this->camera.image_height);
  parser.addParam("/config/camera/fx", &fx);
  parser.addParam("/config/camera/fy", &fy);
  parser.addParam("/config/camera/cx", &cx);
  parser.addParam("/config/camera/cy", &cy);
  parser.addParam("/config/camera/hz", &this->camera.hz);
  parser.addParam("/config/feature_points/nb_features", &this->nb_features);
  parser.addParam("/config/feature_points/x/min", &this->feature_x_bounds(0));
  parser.addParam("/config/feature_points/x/max", &this->feature_x_bounds(1));
  parser.addParam("/config/feature_points/y/min", &this->feature_y_bounds(0));
  parser.addParam("/config/feature_points/y/max", &this->feature_y_bounds(1));
  parser.addParam("/config/feature_points/z/min", &this->feature_z_bounds(0));
  parser.addParam("/config/feature_points/z/max", &this->feature_z_bounds(1));
  if (parser.load(config_file) != 0) {
    log_error("Failed to load [%s]!", config_file.c_str());
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

void calculate_circle_angular_velocity(double r, double v, double &w) {
  double dist, time;

  dist = 2 * M_PI * r;
  time = dist / v;
  w = (2 * M_PI) / time;
}

int VOTestDataset::generateRandom3DFeatures(MatX &features) {
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

int VOTestDataset::record3DFeatures(const std::string &output_path,
                                  const MatX &features) {
  return mat2csv(output_path,
                 features.block(0, 0, 3, features.cols()).transpose());
}

int VOTestDataset::recordObservedFeatures(
  double time,
  const Vec3 &x,
  const std::string &output_path,
  std::vector<std::pair<Vec2, Vec3>> &observed) {
  std::ofstream outfile(output_path);
  Vec2 f_2d;
  Vec3 f_3d;

  // open file
  if (outfile.good() != true) {
    log_error("Failed to open file [%s] to record observed features!",
              output_path.c_str());
    return -1;
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

  return 0;
}

int VOTestDataset::generateTestData(const std::string &save_path) {
  int retval;
  Vec3 x, t, rpy;
  Vec2 u;
  MatX features;
  double w, dt, time;
  std::ofstream output_file;
  std::ofstream index_file;
  std::vector<std::pair<Vec2, Vec3>> observed;
  std::ostringstream oss;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // mkdir calibration directory
  retval = mkdir(save_path.c_str(), ACCESSPERMS);
  if (retval != 0) {
    switch (errno) {
      case EACCES:
        log_error(MKDIR_PERMISSION_DENIED, save_path.c_str());
        break;
      case ENOTDIR: log_error(MKDIR_INVALID, save_path.c_str()); break;
      case EEXIST: log_error(MKDIR_EXISTS, save_path.c_str()); break;
      default: log_error(MKDIR_FAILED, save_path.c_str()); break;
    }
    return -2;
  }

  // setup
  output_file.open(save_path + "/state.dat");
  prep_header(output_file);
  index_file.open(save_path + "/index.dat");
  calculate_circle_angular_velocity(0.5, 1.0, w);
  this->generateRandom3DFeatures(features);
  this->record3DFeatures("/tmp/test/features.dat", features);

  // initialize states
  dt = 0.01;
  time = 0.0;
  x << 0.0, 0.0, 0.0;
  u << 1.0, w;

  for (int i = 0; i < 300; i++) {
    // update state
    x = two_wheel_model(x, u, dt);
    time += dt;

    // check features
    rpy << 0.0, 0.0, x(2);
    t << x(0), x(1), 0.0;
    if (this->camera.checkFeatures(dt, features, rpy, t, observed) == 0) {
      oss.str("");
      oss << "/tmp/test/observed_" << this->camera.frame << ".dat";
      this->recordObservedFeatures(time, x, oss.str(), observed);

      index_file << oss.str() << std::endl;
    }

    // record state
    record_observation(output_file, x);
  }

  // clean up
  output_file.close();
  index_file.close();
  return 0;
}

}  // end of wave namespace
