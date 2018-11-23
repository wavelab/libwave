
// This is to evaluate data from the moose, lidar is NOT pre-corrected

#include "../include/kitti_utility_methods.hpp"
#include "../../tests/data/include/config_utils.hpp"

int main(int argc, char **argv) {
    if (argc < 6) {
        throw std::runtime_error(
                "Must be run with at least 5 arguments: \n 1. Full path to data \n 2. Full path to configuration files \n 3. 1 "
                "or 0 to indicate whether a visualizer should be run \n"
                "4. Starting frame \n 5. Ending frame (or set -1 for no limit)");
    }
    wave::VecE<wave::PoseVel> prev_odom_trajectory;
    if (argc == 7) {
        std::string prev_path(argv[6]);
        LOG_INFO("%s", prev_path.c_str());
        fstream prev_traj_file(prev_path, ios::in);
        std::string line_string;
        while(getline(prev_traj_file, line_string)) {
            std::stringstream ss(line_string);
            std::vector<double> vals;
            for (uint32_t i = 0; i < 12; ++i) {
                double new_val;
                ss >> new_val;
                vals.emplace_back(new_val);
            }
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> raw_mat(vals.data());
            wave::Transformation<> T;
            wave::PoseVel pose_vel;
            pose_vel.pose.storage = raw_mat;
            vals.clear();
            for (uint32_t i = 0; i < 6; ++i) {
                double new_val;
                ss >> new_val;
                vals.emplace_back(new_val);
            }
            Eigen::Map<wave::Vec6> velocity(vals.data());
            pose_vel.vel = velocity;
            prev_odom_trajectory.emplace_back(pose_vel);
        }
    }

    std::string data_path(argv[1]);
    std::string config_path(argv[2]);
    bool run_viz = std::stoi(argv[3]) == 1;
    if (data_path.back() != '/') {
        data_path = data_path + '/';
    }
    if (config_path.back() != '/') {
        config_path = config_path + '/';
    }

    int start_frame = std::stoi(argv[4]);
    int end_frame = std::stoi(argv[5]);
    if (end_frame < 0) {
        end_frame = std::numeric_limits<int>::max();
    }

    wave::Transformation<> T_L_G, T_G_L;
    T_L_G.storage << 0.0146392923884,0.999713733025,0.0189246695892,-1.12495691821,
        -0.999886807886,0.0145708401517,0.00374993783733,0.0726605553018,
        0.00347311601847,-0.0189774239023,0.999813880103,-1.25;
    T_G_L = T_L_G.transformInverse();

    wave::ConfigParser int_parser;
    wave::MatX intensity_map;
    int_parser.addParam("intensity_map", &intensity_map);
    int_parser.load(config_path + "moose_intensity_map.yaml");

    wave::LaserOdomParams params;
    wave::FeatureExtractorParams feature_params;
    loadFeatureParams(config_path, "features.yaml", feature_params);
    setupFeatureParameters(feature_params);
    LoadParameters(config_path, "odom.yaml", params);
    params.n_ring = 32;
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

    std::string gtpath = data_path + "gps.txt";
    std::string lidar_path = data_path + "lidar/";

    // set up pointcloud iterators
    boost::filesystem::path p(lidar_path);
    std::vector<boost::filesystem::path> v;
    std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
    std::sort(v.begin(), v.end());

    // timesteps
    fstream lidar_stamps(data_path + "lidar_times.txt", ios::in);

    if (!lidar_stamps.good()) {
        throw std::runtime_error("file structure not as expected for timestamps");
    }

    std::vector<wave::TimeType> times;
    std::string cur_stamp;
    while (std::getline(lidar_stamps, cur_stamp)) {
        std::stringstream ss(cur_stamp);
        uint64_t microseconds;
        ss >> microseconds;
        wave::TimeType new_stamp;
        new_stamp = new_stamp + std::chrono::microseconds(microseconds);
        times.emplace_back(new_stamp);
    }

    fstream ground_truth_file(gtpath, ios::in);

    wave::VecE<wave::PoseStamped> T_L1_Lx_trajectory;
    wave::Transformation<> T_L1_O;

    if (!ground_truth_file.good()) {
        LOG_INFO("Ground truth not available");
    } else {
        std::string line_string;
        while(true) {
            std::getline(ground_truth_file, line_string);
            std::stringstream ss(line_string);
            if (ss.rdbuf()->in_avail() < 1) {
                break;
            }
            wave::PoseStamped T_L1_Lx;
            int64_t nanoseconds;
            ss >> nanoseconds;
            wave::TimeType new_stamp;
            auto duration = std::chrono::nanoseconds(nanoseconds);
            T_L1_Lx.stamp = new_stamp + duration;
            std::vector<double> vals;
            for (uint32_t i = 0; i < 12; ++i) {
                double new_val;
                ss >> new_val;
                vals.emplace_back(new_val);
            }
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> raw_mat(vals.data());
            wave::Transformation<> T_O_Gx(raw_mat);

            if (T_L1_Lx_trajectory.empty()) {
                T_L1_O.storage = raw_mat;
                T_L1_O.invert();
                T_L1_O = T_L_G * T_L1_O;
            }
            T_L1_Lx.pose = T_L1_O * T_O_Gx * T_G_L;
            T_L1_Lx_trajectory.emplace_back(T_L1_Lx);
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

    std::function<void()> func;
    if (prev_odom_trajectory.empty()) {
        func = [&]() { updateVisualizer(&odom, display, &odom_trajectory, &lengths); };
    } else {
        func = [&]() { updateVisualizer(&odom, display, &odom_trajectory, &lengths, &prev_odom_trajectory); };
    }
    odom.registerOutputFunction(func);

    uint32_t scan_index = 0;
    std::vector<int> mapping(256);
    std::iota(mapping.begin(), mapping.end(), 0);

    bool first_time = true;
    wave::TimeType first_stamp;

    std::vector<long int> frequencies(256);
    std::fill(frequencies.begin(), frequencies.end(), 0);
    long int binned_frequencies = 0;

    for (auto iter = v.begin(); iter != v.end(); ++iter) {
        if (scan_index < start_frame) {
            scan_index++;
            continue;
        }
        if (scan_index >= end_frame) {
            break;
        }
        fstream cloud_file;

        cloud_file.open(iter->string(), ios::in | ios::binary);

        const auto &time_t = times.at(scan_index);

        if (first_time) {
            first_time = false;
            first_stamp = time_t;
        }

        bool first_point = true;

        std::vector<int> intensities;
        intensities.clear();
        while (cloud_file.good() && !cloud_file.eof()) {
            float raw_intensity;
            std::vector<wave::PointXYZIR> pt_vec(1);
            int32_t nanosec_offset;
            float ignore[3];

            cloud_file.read((char *) pt_vec.front().data, 3 * sizeof(float));
            cloud_file.read((char *) &raw_intensity, sizeof(float));
            cloud_file.read((char *) ignore, 3 * sizeof(float));
            cloud_file.read((char *) &(pt_vec.front().ring), sizeof(uint16_t));
            cloud_file.read((char *) &nanosec_offset, sizeof(int32_t));

            if (std::isnan(pt_vec.front().x) || std::isnan(raw_intensity)) {
                continue;
            }

//            pt_vec.front().intensity = raw_intensity > 40 ? raw_intensity : 40;
            pt_vec.front().intensity = (float) (intensity_map((int)(raw_intensity), 0)) / 255.0f;

            int intensity = (int)(raw_intensity);
            if (intensity < 0 || intensity > 255) {
                throw std::runtime_error("ahhh)");
            }
            intensities.emplace_back(intensity);

            wave::TimeType pt_time = times.at(scan_index) + std::chrono::nanoseconds(nanosec_offset);

            if (first_point) {
                odom.addPoints(pt_vec, 0, pt_time);
                first_point = false;
            } else {
                odom.addPoints(pt_vec, 20000, pt_time);
            }
        }

        for (const auto &intensity : intensities) {
            frequencies.at(intensity)++;
            binned_frequencies++;
        }

        cloud_file.close();
        ++scan_index;
        if (scan_index % 10 == 0 || scan_index == times.size()) {
            std::cout << "\rFinished with scan " << std::to_string(scan_index) << "/"
                      << std::to_string(times.size()) << std::flush;
        }
    }

    std::vector<double> pdf(256), cdf(256);
    for (uint32_t i = 0; i < frequencies.size(); ++i) {
        pdf.at(i) = ((double) (frequencies.at(i)) / (double)(binned_frequencies));
        cdf.at(i) = pdf.at(i);
        if(i > 0) {
            cdf.at(i) += cdf.at(i-1);
        }
        mapping.at(i) = (int)(cdf.at(i) * 255);
    }

    std::ofstream map_file("mapping.txt");
    for (const auto &map : mapping) {
        map_file << map << "\n";
    }
    map_file.close();

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

    auto original_trajectory = odom_trajectory;

    if (iter != T_L1_Lx_trajectory.end()) {
        for (auto &odom_state : odom_trajectory) {
            odom_state.pose = iter->pose * odom_state.pose;
        }
    }

    const static Eigen::IOFormat Format(
            Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ");

    // kitti error eval program expects movement in xz plane, so need to rotate
    wave::Transformation<> T_C_L;
    T_C_L.storage << 0, -1.0, 0, 0,
                                  0, 0, -1.0, 0,
                                  1, 0, 0, 0;

    wave::Transformation<> T_L_C = T_C_L.transformInverse();

    if (T_L1_Lx_trajectory.empty()) {
        plotResults(odom_trajectory);
    } else {
        plotResults(T_L1_Lx_trajectory, odom_trajectory);
        plotError(T_L1_Lx_trajectory, odom_trajectory);

        ofstream gt_file("ground_truth.txt");

        for (const auto &time : times) {
            wave::Transformation<> T_C1_Cx;
            auto gt = std::find_if(T_L1_Lx_trajectory.begin(), T_L1_Lx_trajectory.end(), [&time](const wave::PoseStamped &state){

                if (state.stamp >= time) {
                    if (state.stamp - time < std::chrono::milliseconds(10)) {
                        return true;
                    }
                } else {
                    if (time - state.stamp < std::chrono::milliseconds(10)) {
                        return true;
                    }
                }
                return false;
            });

            if (gt == T_L1_Lx_trajectory.end()) {
                continue;
            }
            auto &truth_val = *gt;
            T_C1_Cx = T_C_L * truth_val.pose * T_L_C;
            gt_file << T_C1_Cx.storage.format(Format) << "\n";
        }
    }
    plotTrackLengths(lengths);

    ofstream output_file("lidar_traj.txt");

    if (prev_odom_trajectory.empty()) {
        wave::Transformation<> T_C1_Cx;
        for (const auto &T_L1_Lx : odom_trajectory) {
            T_C1_Cx = T_C_L * T_L1_Lx.pose * T_L_C;
            output_file << T_C1_Cx.storage.format(Format) << "\n";
        }
    }

    odom.~LaserOdom();

    if (run_viz) {
        display->stopSpin();
        delete display;
    }

    return 0;
}
