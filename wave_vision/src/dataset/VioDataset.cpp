#include "wave/vision/dataset/VioDataset.hpp"

#include <ctime>                 // for localtime
#include <iomanip>               // for put_time
#include <boost/filesystem.hpp>  // for create_directories
#include "wave/utils/config.hpp"
#include "wave/vision/dataset/VoDataset.hpp"

namespace wave {

// Helper functions used only in this file
namespace {

// Given a time point, produce a string matching "2011-09-26 14:02:22.484109563"
std::string formatTimestamp(
  const std::chrono::system_clock::time_point &system_time_point) {
    auto t = std::chrono::system_clock::to_time_t(system_time_point);

    // time_t does not have fractional seconds, so need to do this extra
    auto ns_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(
      system_time_point.time_since_epoch());
    auto ns = ns_since_epoch.count() % static_cast<int>(1e9);

    std::stringstream ss;
    ss << std::put_time(std::gmtime(&t), "%Y-%m-%d %H:%M:%S.");
    ss << std::setw(9) << std::setfill('0') << ns;
    return ss.str();
}

// Read a string matching "2011-09-26 14:02:22.484109563" from an input stream,
// and produce a time point. Return true on success
bool readTimepointFromStream(
  std::istream &in, std::chrono::system_clock::time_point &time_point) {
    // std::get_time does not have fractional seconds, so it's not trivial.
    // First we parse the part up to the dot, then the fractional seconds
    double fractional_secs;
    std::tm tm;
    in >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
    in >> fractional_secs;
    time_point = std::chrono::system_clock::from_time_t(std::mktime(&tm));

    // Add fractional seconds
    const auto d =
      std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::duration<double>(fractional_secs));
    time_point += d;

    // If the stream ended, we will return false
    return in.good();
}


// Data contained in the kitti format data files
struct OxtsData {
    double lat, lon, alt, roll, pitch, yaw;
    double vn, ve, vf, vl, vu, ax, ay, az;
    double af, al, au, wx, wy, wz, wf, wl, wu;
    double pos_accuracy, vel_accuracy;
    int navstat, numsats, posmode, velmode, orimode;
};

std::istream &operator>>(std::istream &in, OxtsData &o) {
    in >> o.lat >> o.lon >> o.alt;
    in >> o.roll >> o.pitch >> o.yaw;
    in >> o.vn >> o.ve >> o.vf;
    in >> o.vl >> o.vu;
    in >> o.ax >> o.ay >> o.az;
    in >> o.af >> o.al >> o.au;
    in >> o.wx >> o.wy >> o.wz;
    in >> o.wf >> o.wl >> o.wu;
    in >> o.pos_accuracy >> o.vel_accuracy;
    in >> o.navstat >> o.numsats >> o.posmode >> o.velmode >> o.orimode;
    return in;
}

std::ostream &operator<<(std::ostream &os, const OxtsData &o) {
    os << o.lat << " " << o.lon << " " << o.alt << " ";
    os << o.roll << " " << o.pitch << " " << o.yaw << " ";
    os << o.vn << " " << o.ve << " " << o.vf << " ";
    os << o.vl << " " << o.vu << " ";
    os << o.ax << " " << o.ay << " " << o.az << " ";
    os << o.af << " " << o.al << " " << o.au << " ";
    os << o.wx << " " << o.wy << " " << o.wz << " ";
    os << o.wf << " " << o.wl << " " << o.wu << " ";
    os << o.pos_accuracy << " " << o.vel_accuracy << " ";
    os << o.navstat << " " << o.numsats << " " << o.posmode << " " << o.velmode
       << " " << o.orimode;
    return os;
}

// Convert lat/lon coordinates to local cartsian using simple spherical model
// (good enough for points close to the origin)
// lla vectors are lat (degrees), lon (degrees), alt (m)
Vec3 enuFromLla(Vec3 lla_datum, Vec3 lla) {
    const auto radius = 6378137;
    const auto l = radius * M_PI / 180;
    const auto x = (lla[1] - lla_datum[1]) * l * cos(lla[0] * M_PI / 180);
    const auto y = (lla[0] - lla_datum[0]) * l;
    return Vec3{x, y, lla[2]};
}

// Convert local cartesian to lat/lon using simple model
Vec3 llaFromEnu(Vec3 lla_datum, Vec3 enu) {
    const auto radius = 6378137;
    const auto l = radius * M_PI / 180;
    const auto lat = lla_datum[0] + enu[1] / l;
    const auto lon = lla_datum[1] + enu[0] / cos(lat * M_PI / 180) / l;
    return Vec3{lat, lon, enu[2]};
}

// For each time in the container, write a timestamp
void writeTimestampsToFile(const VioDataset::PoseContainer &container,
                           const std::string &output_path) {
    std::ofstream timestamps_file{output_path};

    // Choose an arbitrary start time
    // @todo maybe we will care about absolute time in future
    const auto start_time = std::chrono::system_clock::now();
    // Let the first measurement in the container occur at that start_time
    // (with the current setup, we have to convert from steady_clock to
    // system_clock values - todo?)
    const auto steady_start_time = container.begin()->time_point;

    for (const auto &meas : container) {
        auto time_point = start_time + (meas.time_point - steady_start_time);
        timestamps_file << formatTimestamp(time_point) << std::endl;
    }
}

// load calibration files into dataset
void loadCalibration(const std::string &input_dir, VioDataset &dataset) {
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
}

// Load landmarks into dataset
void loadLandmarks(const std::string &input_dir, VioDataset &dataset) {
    std::ifstream landmarks_file{input_dir + "/landmarks.txt"};
    for (LandmarkId id; landmarks_file >> id;) {
        auto landmark_pos = matrixFromStream<3, 1>(landmarks_file);
        dataset.landmarks.emplace(id, landmark_pos);
    }
}

// Produce a filename the file with the format "0000000000.txt"
std::string paddedTxtFilename(int i) {
    std::stringstream ss;
    ss.fill('0');
    ss.width(10);
    ss << i << ".txt";
    return ss.str();
}

// Load gps and imu measurements into dataset
void loadPoses(const std::string &input_dir, VioDataset &dataset) {
    const auto timestamps_filename = input_dir + "/oxts/timestamps.txt";
    auto timestamps_file = std::ifstream{timestamps_filename};

    if (!timestamps_file) {
        throw std::runtime_error("Could not read " + timestamps_filename);
    }

    const auto data_dir = input_dir + "/oxts/data";
    std::chrono::system_clock::time_point system_start_time, system_time_point;
    const auto start_time = std::chrono::steady_clock::now();  // arbitrary

    for (int i = 0; readTimepointFromStream(timestamps_file, system_time_point);
         ++i) {
        // load the file with format "0000000000.txt"
        const auto filename = data_dir + "/" + paddedTxtFilename(i);
        auto data_file = std::ifstream{filename};
        OxtsData data;
        data_file >> data;
        if (!data_file) {
            throw std::runtime_error("Could not read " + filename);
        }

        // Use the first position as the datum, and first time as the reference
        if (i == 0) {
            dataset.lla_datum << data.lat, data.lon, data.alt;
            system_start_time = system_time_point;
        }
        const auto time_point =
          start_time + (system_time_point - system_start_time);


        VioDataset::PoseValue pose;
        pose.G_p_GI =
          enuFromLla(dataset.lla_datum, Vec3{data.lat, data.lon, data.alt});
        pose.R_GI.setFromEulerXYZ(Vec3{data.roll, data.pitch, data.yaw});
        dataset.poses.emplace(time_point, VioDataset::PoseSensor::Pose, pose);

        if (i == 0) {
            std::cerr << "\n the first pose is " << pose.G_p_GI.transpose()
                      << std::endl;
        }

        VioDataset::ImuValue imu;
        imu.I_vel_GI << data.vf, data.vl, data.vu;
        imu.I_ang_vel_GI << data.wf, data.wl, data.wu;
        dataset.imu_measurements.emplace(
          time_point, VioDataset::ImuSensor::Imu, imu);
    }
}


}  // namespace

void VioDataset::outputToDirectory(const std::string &output_dir) const {
    boost::filesystem::create_directories(output_dir);

    // Landmarks - not in kitti. Output using existing vo format
    VoDataset vo;
    vo.landmarks = this->landmarks;
    vo.outputLandmarks(output_dir + "/landmarks.txt");

    this->outputCalibration(output_dir);
    this->outputPoses(output_dir + "/oxts");
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

void VioDataset::outputPoses(const std::string &output_dir) const {
    boost::filesystem::create_directories(output_dir + "/data");

    writeTimestampsToFile(this->poses, output_dir + "/timestamps.txt");

    int i = 0;
    auto imu_iter = this->imu_measurements.cbegin();
    auto pose_iter = this->poses.cbegin();

    // Note we assume they are the same length.
    while (imu_iter != this->imu_measurements.cend() &&
           pose_iter != this->poses.cend()) {
        std::ofstream data_file{output_dir + "/data/" + paddedTxtFilename(i)};

        // We only write to some of the values right now. The others are zero.
        auto data = OxtsData{};

        const PoseValue &v = pose_iter->value;
        auto lla = llaFromEnu(this->lla_datum, v.G_p_GI);
        data.lat = lla[0];
        data.lon = lla[1];
        data.alt = lla[2];

        Vec3 euler = v.R_GI.toRotationMatrix().eulerAngles(0, 1, 2);
        data.roll = euler[0];
        data.pitch = euler[1];
        data.yaw = euler[2];

        ImuValue m = imu_iter->value;
        data.vf = m.I_vel_GI[0];
        data.vl = m.I_vel_GI[1];
        data.vu = m.I_vel_GI[2];
        data.wf = m.I_ang_vel_GI[0];
        data.wl = m.I_ang_vel_GI[1];
        data.wu = m.I_ang_vel_GI[2];

        data_file << std::setprecision(12) << data << std::endl;

        ++pose_iter;
        ++imu_iter;
        ++i;
    }
}

VioDataset VioDataset::loadFromDirectory(const std::string &input_dir) {
    VioDataset dataset;

    loadCalibration(input_dir, dataset);
    loadLandmarks(input_dir, dataset);
    loadPoses(input_dir, dataset);

    return dataset;
}

}  // namespace wave
