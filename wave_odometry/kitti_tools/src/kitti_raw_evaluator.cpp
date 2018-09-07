// This is to evaluate the raw uncorrected scans from kitti
// todo: Find the calibration between OXT and Lidar so that ground truth is actually ground truth

#include "wave/odometry/laser_odom.hpp"
#include "../include/kitti_utility_methods.hpp"
#include "../../tests/data/include/config_utils.hpp"

int main(int argc, char **argv) {
    if (argc != 4) {
        throw std::runtime_error(
          "Must be run with only 3 arguments: \n 1. Full path to data \n 2. Full path to configuration files \n 3. 1 "
          "or 0 to indicate whether a visualizer should be run");
    }
    std::string data_path(argv[1]);
    std::string config_path(argv[2]);
    if (data_path.back() != '/') {
        data_path = data_path + '/';
    }
    if (config_path.back() != '/') {
        config_path = config_path + '/';
    }

    wave::ConfigParser parser;
    wave::MatX cutoff_angles;
    parser.addParam("cutoff_angles", &cutoff_angles);
    parser.load(config_path + "kitti_angles.yaml");

    wave::LaserOdomParams params;
    wave::FeatureExtractorParams feature_params;
    wave::loadFeatureParams(config_path, "features.yaml", feature_params);
    setupFeatureParameters(feature_params);
    LoadParameters(config_path, "odom.yaml", params);
    params.n_ring = 64;
    wave::TransformerParams transformer_params;
    transformer_params.traj_resolution = params.num_trajectory_states;
    transformer_params.delRTol = 100.0f;
    transformer_params.delVTol = 100.0f;
    transformer_params.delWTol = 100.0f;

    wave::loadBinnerParams(config_path, "bin_config.yaml", params.binner_params);

    wave::LaserOdom odom(params, feature_params, transformer_params);
    wave::PointCloudDisplay *display;

    bool run_viz = std::stoi(argv[3]) == 1;
    if (run_viz) {
        display = new wave::PointCloudDisplay("Kitti Eval", 0.2, 3, 2);
        display->startSpin();
    } else {
        display = nullptr;
    }

    // set up pointcloud iterators
    boost::filesystem::path p(data_path + "velodyne_points/data");
    std::vector<boost::filesystem::path> v;
    std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
    std::sort(v.begin(), v.end());

    // timesteps
    fstream timestamps_start(data_path + "velodyne_points/timestamps_start.txt", ios::in);
    fstream timestamps_end(data_path + "velodyne_points/timestamps_end.txt", ios::in);

    if (!timestamps_start.good() || !timestamps_end.good()) {
        throw std::runtime_error("file structure not as expected for timestamps");
    }

    std::vector<wave::TimeType> time_start, time_end;
    std::string cur_start_stamp, cur_end_stamp;
    while (std::getline(timestamps_start, cur_start_stamp) && std::getline(timestamps_end, cur_end_stamp)) {
        time_start.emplace_back(wave::parseTime(cur_start_stamp));
        time_end.emplace_back(wave::parseTime(cur_end_stamp));
    }

    wave::VecE<wave::PoseVelStamped> oxt_trajectory, odom_trajectory;
    fillGroundTruth(oxt_trajectory, data_path);

    std::vector<wave::TrackLengths> lengths(5);
    for (auto &length : lengths) {
        length.lengths.resize(params.n_window + 1);
    }

    auto func = [&]() { updateVisualizer(&odom, display, &odom_trajectory, &lengths); };
    odom.registerOutputFunction(func);

    int pt_index = 0;

    unsigned long counter = 0;
    bool binary_format = false;
    uint16_t ring_index = 0;

    uint32_t scan_index = 0;
    std::vector<int> mapping(101);
    std::iota(mapping.begin(), mapping.end(), 0);
    for (auto iter = v.begin(); iter != v.end(); ++iter) {
        fstream cloud_file;
        if (iter->string().substr(iter->string().find_last_of('.') + 1) == "bin") {
            binary_format = true;
            cloud_file.open(iter->string(), ios::in | ios::binary);
        } else {
            cloud_file.open(iter->string(), ios::in);
        }
        const auto &start_t = time_start.at(counter);
        const auto &end_t = time_end.at(counter);
        ++counter;
        std::chrono::nanoseconds diff = end_t - start_t;

        ring_index = 0;
        double prev_azimuth = 0;
        bool first_point = true;
        std::vector<int> intensities;
        intensities.clear();
        while (cloud_file.good() && !cloud_file.eof()) {
            std::vector<wave::PointXYZIR> pt_vec(1);
            float raw_intensity;
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

            float new_intensity = (float) (mapping.at((int)(100 * raw_intensity))) / 100.0f;
            pt_vec.front().intensity = raw_intensity > 0.5 ? raw_intensity : 0.5;

            intensities.emplace_back((int)(100 * raw_intensity));

            auto azimuth = (std::atan2(pt_vec.front().y, pt_vec.front().x));
            azimuth < 0 ? azimuth = (float) (azimuth + 2.0 * M_PI) : azimuth;
            double from_start = azimuth - cutoff_angles(ring_index, 0);
            from_start < 0 ? from_start = from_start + 2.0 * M_PI : from_start;

            double time_scaling = (2 * M_PI - from_start) / (2 * M_PI);

            double nanoFP = time_scaling * diff.count();
            std::chrono::nanoseconds scaled_diff((long) nanoFP);

            wave::TimeType stamp = start_t + scaled_diff;

            if (prev_azimuth > azimuth + 0.2) {
                ++ring_index;
            }
            prev_azimuth = azimuth;

            pt_vec.front().ring = ring_index;

            if (ring_index < 64) {
                if (first_point) {
                    odom.addPoints(pt_vec, 0, stamp);
                    pt_index = 0;
                    first_point = false;
                } else {
                    odom.addPoints(pt_vec, 3000, stamp);
                    ++pt_index;
                }
            }
        }

        std::vector<int> frequencies(101);
        std::fill(frequencies.begin(), frequencies.end(), 0);
        for (const auto &intensity : intensities) {
            frequencies.at(intensity)++;
        }
        std::vector<double> pdf(101), cdf(101);

        for (uint32_t i = 0; i < frequencies.size(); ++i) {
            pdf.at(i) = ((double) (frequencies.at(i)) / (double)(intensities.size()));
            cdf.at(i) = pdf.at(i);
            if(i > 0) {
                cdf.at(i) += cdf.at(i-1);
            }
            mapping.at(i) = (int)(cdf.at(i) * 100);
        }

        cloud_file.close();
        ++scan_index;
        if (scan_index % 10 == 0 || scan_index == oxt_trajectory.size()) {
            std::cout << "\rFinished with scan " << std::to_string(scan_index) << "/"
                      << std::to_string(oxt_trajectory.size()) << std::flush;
        }
    }
    plotResults(oxt_trajectory, odom_trajectory);
    plotTrackLengths(lengths);

    odom.~LaserOdom();

    if (run_viz) {
        display->stopSpin();
        delete display;
    }

    return 0;
}