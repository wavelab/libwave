// This is to evaluate the odometry sequences from kitti. Unlike the
// raw scans, they come pre-corrected for motion distortion

#include "../include/kitti_utility_methods.hpp"
#include "../../tests/data/include/config_utils.hpp"

int main(int argc, char **argv) {
    if (argc != 7) {
        throw std::runtime_error(
                "Must be run with only 6 arguments: \n 1. Full path to data \n 2. Full path to configuration files \n 3. 1 "
                "or 0 to indicate whether a visualizer should be run \n 4. Sequence to run. \n"
                "5. Starting frame \n 6. Ending frame (or set -1 for no limit)");
    }
    std::string data_path(argv[1]);
    std::string config_path(argv[2]);
    bool run_viz = std::stoi(argv[3]) == 1;
    std::string sequence(argv[4]);
    if (data_path.back() != '/') {
        data_path = data_path + '/';
    }
    if (config_path.back() != '/') {
        config_path = config_path + '/';
    }

    int start_frame = std::stoi(argv[5]);
    int end_frame = std::stoi(argv[6]);
    if (end_frame < 0) {
        end_frame = std::numeric_limits<int>::max();
    }

    wave::ConfigParser parser;
    wave::MatX cutoff_angles;
    parser.addParam("cutoff_angles", &cutoff_angles);
    parser.load(config_path + "kitti_angles.yaml");

    wave::ConfigParser int_parser;
    wave::MatX intensity_map;
    int_parser.addParam("intensity_map", &intensity_map);
    int_parser.load(config_path + "kitti_intensity_map.yaml");

    wave::LaserOdomParams params;
    wave::FeatureExtractorParams feature_params;
    loadFeatureParams(config_path, "features.yaml", feature_params);
    setupFeatureParameters(feature_params);
    LoadParameters(config_path, "odom.yaml", params);
    params.n_ring = 64;
    wave::TransformerParams transformer_params;
    transformer_params.traj_resolution = params.num_trajectory_states;
    transformer_params.delRTol = 100.0f;
    transformer_params.delVTol = 100.0f;
    transformer_params.delWTol = 100.0f;

    wave::loadBinnerParams(config_path, "bin_config.yaml", params.binner_params);

    wave::PointCloudDisplay *display;

    if (run_viz) {
        display = new wave::PointCloudDisplay("Kitti Eval", 0.2, 3, 2);
        display->startSpin();
    } else {
        display = nullptr;
    }

    std::string gtpath = data_path + "poses/" + sequence + ".txt";
    data_path = data_path + "sequences/" + sequence + "/";
    std::string calibration_path = data_path + "calib.txt";

    // set up pointcloud iterators
    boost::filesystem::path p(data_path + "velodyne");
    std::vector<boost::filesystem::path> v;
    std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
    std::sort(v.begin(), v.end());

    // timesteps
    fstream timestamps(data_path + "times.txt", ios::in);

    if (!timestamps.good()) {
        throw std::runtime_error("file structure not as expected for timestamps");
    }

    std::vector<wave::TimeType> times;
    std::string cur_stamp;
    while (std::getline(timestamps, cur_stamp)) {
        std::stringstream ss(cur_stamp);
        double seconds;
        ss >> seconds;
        wave::TimeType new_stamp;
        auto micros = static_cast<uint64_t>(seconds * 1e6);
        new_stamp = new_stamp + std::chrono::microseconds(micros);
        times.emplace_back(new_stamp);
    }

    fstream ground_truth_file(gtpath, ios::in);
    fstream calibration_file(calibration_path, ios::in);

    if (!calibration_file.good()) {
        throw std::runtime_error("Unable to find calibration");
    }

    wave::Transformation<> T_CAM_LIDAR;

    {
        std::string data_input;
        for (uint32_t i = 0; i < 5; ++i) {
            std::getline(calibration_file, data_input);
        }
        std::stringstream ss(data_input);
        std::string junk;
        ss >> junk;
        std::vector<double> vals;
        while (ss.rdbuf()->in_avail() > 0) {
            double new_val;
            ss >> new_val;
            vals.emplace_back(new_val);
        }
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> raw_mat(vals.data());
        T_CAM_LIDAR = wave::Transformation<decltype(raw_mat), false>(raw_mat);
    }

    wave::VecE<wave::PoseStamped> T_L1_Lx_trajectory;

    wave::Transformation<> T_L1_O;

    if (!ground_truth_file.good()) {
        LOG_INFO("Ground truth not available");
    } else {
        std::string line_string;
        T_L1_Lx_trajectory.resize(times.size());
        for (uint32_t frame_id = 0; frame_id < times.size(); ++frame_id) {
            std::getline(ground_truth_file, line_string);
            std::stringstream ss(line_string);
            std::vector<double> vals;
            while (ss.rdbuf()->in_avail() > 0) {
                double new_val;
                ss >> new_val;
                vals.emplace_back(new_val);
            }
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> raw_mat(vals.data());
            wave::Transformation<> T_O_cam(raw_mat);
            wave::Transformation<> T_O_Lx = T_O_cam * T_CAM_LIDAR;
            if (frame_id == 0) {
                T_L1_O = T_O_Lx.transformInverse();
            }
            T_L1_Lx_trajectory.at(frame_id).pose = T_L1_O * T_O_Lx;
            T_L1_Lx_trajectory.at(frame_id).stamp = times.at(frame_id);
        }
        params.initial_velocity = T_L1_Lx_trajectory.at(1).pose.manifoldMinus(T_L1_Lx_trajectory.at(0).pose);
        double delT = std::chrono::duration<double>(T_L1_Lx_trajectory.at(1).stamp - T_L1_Lx_trajectory.at(0).stamp).count();
        params.initial_velocity = params.initial_velocity / delT;
    }

    wave::VecE<wave::PoseVelStamped> odom_trajectory;

    std::vector<wave::TrackLengths> lengths(5);
    for (auto &length : lengths) {
        length.lengths.resize(params.n_window + 1);
    }

    wave::LaserOdom odom(params, feature_params, transformer_params);
    auto func = [&]() { updateVisualizer(&odom, display, &odom_trajectory, &lengths); };
    odom.registerOutputFunction(func);

    bool binary_format = false;
    uint16_t ring_index = 0;

    uint32_t scan_index = 0;
    std::vector<int> mapping(101);
    std::iota(mapping.begin(), mapping.end(), 0);

    bool first_time = true;
    wave::TimeType first_stamp;

    std::vector<long int> frequencies(101);
    std::fill(frequencies.begin(), frequencies.end(), 0);
    long int binned_frequencies = 0;

    for (auto iter = v.begin(); iter != v.end(); ++iter) {
        if (scan_index < start_frame) {
            scan_index++;
            continue;
        }
        if (scan_index > end_frame) {
            break;
        }
        fstream cloud_file;
        if (iter->string().substr(iter->string().find_last_of('.') + 1) == "bin") {
            binary_format = true;
            cloud_file.open(iter->string(), ios::in | ios::binary);
        } else {
            cloud_file.open(iter->string(), ios::in);
        }
        const auto &time_t = times.at(scan_index);

        if (first_time) {
            first_time = false;
            first_stamp = time_t;
        }

        ring_index = 0;
        double prev_azimuth = 0;
        bool first_point = true;
        std::vector<int> intensities;
        intensities.clear();
        while (cloud_file.good() && !cloud_file.eof()) {
            float raw_intensity;
            std::vector<wave::PointXYZIR> pt_vec(1);
            if (binary_format) {
                cloud_file.read((char *) pt_vec.front().data, 3 * sizeof(float));
                cloud_file.read((char *) &raw_intensity, sizeof(float));
            } else {
                std::string line;
                std::getline(cloud_file, line);
                std::stringstream ss(line);
                ss >> pt_vec.front().x;
                ss >> pt_vec.front().y;
                ss >> pt_vec.front().z;
                ss >> raw_intensity;
            }
//            float new_intensity = (float) (mapping.at((int)(100 * raw_intensity))) / 100.0f;
            float new_intensity = (float) (intensity_map((int)(100 * raw_intensity), 0)) / 100.0f;
            pt_vec.front().intensity = new_intensity; // new_intensity > 0.9 ? new_intensity : 0.9;

            intensities.emplace_back((int)(100 * raw_intensity));

            auto azimuth = (std::atan2(pt_vec.front().y, pt_vec.front().x));
            azimuth < 0 ? azimuth = (float) (azimuth + 2.0 * M_PI) : azimuth;

            if (prev_azimuth > azimuth + 0.2) {
                ++ring_index;
            }
            prev_azimuth = azimuth;

            pt_vec.front().ring = ring_index;

            if (std::abs(azimuth - M_PI) < 0.1) {
                continue;
            }

            if (ring_index < 64) {
                if (first_point) {
                    odom.addPoints(pt_vec, 0, time_t);
                    first_point = false;
                } else {
                    odom.addPoints(pt_vec, 3000, time_t);;
                }
            }
        }

        for (const auto &intensity : intensities) {
            frequencies.at(intensity)++;
            binned_frequencies++;
        }
        std::vector<double> pdf(101), cdf(101);

        for (uint32_t i = 0; i < frequencies.size(); ++i) {
            pdf.at(i) = ((double) (frequencies.at(i)) / (double)(binned_frequencies));
            cdf.at(i) = pdf.at(i);
            if(i > 0) {
                cdf.at(i) += cdf.at(i-1);
            }
            mapping.at(i) = (int)(cdf.at(i) * 100);
        }

        cloud_file.close();
        ++scan_index;
        if (scan_index % 10 == 0 || scan_index == times.size()) {
            std::cout << "\rFinished with scan " << std::to_string(scan_index) << "/"
                      << std::to_string(times.size()) << std::flush;
        }
    }

    auto iter = std::find_if(T_L1_Lx_trajectory.begin(), T_L1_Lx_trajectory.end(), [&first_stamp](const wave::PoseStamped &state){

       if (state.stamp >= first_stamp) {
           if (state.stamp - first_stamp < std::chrono::milliseconds(10)) {
               return true;
           }
       } else {
           if (first_stamp - state.stamp < std::chrono::milliseconds(10)) {
               return true;
           }
       }
       return false;
    });

    if (iter != T_L1_Lx_trajectory.end()) {
        for (auto &odom_state : odom_trajectory) {
            odom_state.pose = iter->pose * odom_state.pose;
        }
    }

    if (run_viz) {
        if (T_L1_Lx_trajectory.empty()) {
            plotResults(odom_trajectory);
        } else {
            plotResults(T_L1_Lx_trajectory, odom_trajectory);
            plotError(T_L1_Lx_trajectory, odom_trajectory);
        }
        plotTrackLengths(lengths);
    }

    ofstream output_file(sequence + ".txt");
    wave::Transformation<> T_O_L1 = T_L1_O.transformInverse();

    ofstream map_file("mapping.txt");
    for (const auto &map : mapping) {
        map_file << map << "\n";
    }
    map_file.close();

    const static Eigen::IOFormat Format(
            Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ");

    for (const auto &T_L1_Lx : odom_trajectory) {
        wave::Transformation<> T_O_CAM;
        T_O_CAM = T_O_L1 * T_L1_Lx.pose * T_CAM_LIDAR.transformInverse();
        output_file << T_O_CAM.storage.format(Format) << "\n";
    }

    odom.~LaserOdom();

    if (run_viz) {
        display->stopSpin();
        delete display;
    }

    return 0;
}
