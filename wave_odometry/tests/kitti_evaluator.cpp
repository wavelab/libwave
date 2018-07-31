// This is to evaluate kitti sequences

#include <matplotlibcpp.h>

#include "wave/matching/pointcloud_display.hpp"
#include "wave/odometry/laser_odom.hpp"
#include "wave/utils/config.hpp"

//todo Figure out where these helper functions should go (they are shared with tests).
namespace {

void LoadSensorParameters(const std::string &path, const std::string &filename, wave::RangeSensorParams &senparams) {
    wave::ConfigParser parser;
    parser.addParam("rings", &(senparams.rings));
    parser.addParam("sigma_spherical", &(senparams.sigma_spherical));
    parser.addParam("elevation_angles", &(senparams.elevation_angles));

    parser.load(path + filename);
}

void LoadParameters(const std::string &path, const std::string &filename, wave::LaserOdomParams &params) {
    wave::ConfigParser parser;
    parser.addParam("Qc", &(params.Qc));
    parser.addParam("num_trajectory_states", &(params.num_trajectory_states));
    parser.addParam("n_window", &(params.n_window));
    parser.addParam("opt_iters", &(params.opt_iters));
    parser.addParam("max_inner_iters", &(params.max_inner_iters));
    parser.addParam("diff_tol", &(params.diff_tol));
    parser.addParam("solver_threads", &(params.solver_threads));
    parser.addParam("robust_param", &(params.robust_param));
    parser.addParam("max_correspondence_dist", &(params.max_correspondence_dist));
    parser.addParam("max_residual_val", &(params.max_residual_val));
    parser.addParam("min_residuals", &(params.min_residuals));
    parser.addParam("n_ring", &(params.n_ring));

    LoadSensorParameters(path, "sensor_model.yaml", params.sensor_params);

    parser.addParam("azimuth_tol", &(params.azimuth_tol));
    parser.addParam("max_planar_dist_threshold", &(params.max_planar_dist_threshold));
    parser.addParam("max_planar_ang_threshold", &(params.max_planar_ang_threshold));
    parser.addParam("max_linear_dist_threshold", &(params.max_linear_dist_threshold));
    parser.addParam("max_linear_ang_threshold", &(params.max_linear_ang_threshold));
    parser.addParam("ang_scaling_param", &(params.ang_scaling_param));

    parser.addParam("only_extract_features", &(params.only_extract_features));
    parser.addParam("use_weighting", &(params.use_weighting));
    parser.addParam("print_opt_sum", &(params.print_opt_sum));

    parser.load(path + filename);
}

void loadFeatureParams(const std::string &path, const std::string &filename, wave::FeatureExtractorParams &params) {
    wave::ConfigParser parser;

    parser.addParam("variance_window", &(params.variance_window));
    parser.addParam("variance_limit_rng", &(params.variance_limit_rng));
    parser.addParam("variance_limit_int", &(params.variance_limit_int));
    parser.addParam("angular_bins", &(params.angular_bins));
    parser.addParam("min_intensity", &(params.min_intensity));
    parser.addParam("max_intensity", &(params.max_intensity));
    parser.addParam("occlusion_tol", &(params.occlusion_tol));
    parser.addParam("occlusion_tol_2", &(params.occlusion_tol_2));
    parser.addParam("parallel_tol", &(params.parallel_tol));
    parser.addParam("edge_tol", &(params.edge_tol));
    parser.addParam("flat_tol", &(params.flat_tol));
    parser.addParam("int_edge_tol", &(params.int_edge_tol));
    parser.addParam("int_flat_tol", &(params.int_flat_tol));
    parser.addParam("n_edge", &(params.n_edge));
    parser.addParam("n_flat", &(params.n_flat));
    parser.addParam("n_int_edge", &(params.n_int_edge));
    parser.addParam("knn", &(params.knn));
    parser.addParam("key_radius", &(params.key_radius));

    parser.load(path + filename);
}

void setupFeatureParameters(wave::FeatureExtractorParams &param) {
    std::vector<wave::Criteria> edge_high, edge_low, flat, edge_int_high, edge_int_low;
    edge_high.emplace_back(wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::HIGH_POS, &(param.edge_tol)});

    edge_low.emplace_back(wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::HIGH_NEG, &(param.edge_tol)});

    flat.emplace_back(wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::NEAR_ZERO, &(param.flat_tol)});

    edge_int_high.emplace_back(
            wave::Criteria{wave::Signal::INTENSITY, wave::Kernel::FOG, wave::SelectionPolicy::HIGH_POS, &(param.int_edge_tol)});
    edge_int_high.emplace_back(
            wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_high.emplace_back(
            wave::Criteria{wave::Signal::RANGE, wave::Kernel::RNG_VAR, wave::SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    edge_int_low.emplace_back(
            wave::Criteria{wave::Signal::INTENSITY, wave::Kernel::FOG, wave::SelectionPolicy::HIGH_NEG, &(param.int_edge_tol)});
    edge_int_low.emplace_back(wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_low.emplace_back(
            wave::Criteria{wave::Signal::RANGE, wave::Kernel::RNG_VAR, wave::SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    param.feature_definitions.clear();
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_high, &(param.n_edge)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_low, &(param.n_edge)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{flat, &(param.n_flat)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_int_high, &(param.n_int_edge)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_int_low, &(param.n_int_edge)});
}

void updateVisualizer(const wave::LaserOdom *odom, wave::PointCloudDisplay *display) {
    int ptcld_id = 100000;
    display->removeAll();
//    pcl::PointCloud<pcl::PointXYZI>::Ptr viz_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
//    *viz_cloud = odom->undistorted_cld;
//    display->addPointcloud(viz_cloud, ptcld_id, false);
//    ++ptcld_id;

//    for (uint32_t feat_id = 0; feat_id < 3; ++feat_id) {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr viz_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//        *viz_cloud = odom->undis_features.at(feat_id);
//        int viewport_id = feat_id + 1;
//        display->addPointcloud(viz_cloud, ptcld_id, false, viewport_id);
//        ++ptcld_id;
//    }

    for (uint32_t i = 1; i < odom->undistort_trajectory.size(); ++i) {
        pcl::PointXYZ pt1, pt2;
        Eigen::Map<wave::Vec3f> m1(pt1.data), m2(pt2.data);
        m1 = odom->undistort_trajectory.at(i - 1).pose.storage.block<3, 1>(0,3).cast<float>();
        m2 = odom->undistort_trajectory.at(i).pose.storage.block<3, 1>(0,3).cast<float>();
        display->addLine(pt1, pt2, i - 1, i);
    }

    int id = odom->undistort_trajectory.size();

    for (uint32_t i = 0; i < 3; ++i) {
        int viewport_id = 2*i + 1;

        for(const auto &geometry : odom->geometry_landmarks.at(i)) {
            if (i == 2) {
                pcl::PointXYZ pt1, pt2;
                Eigen::Map<wave::Vec3f> m1(pt1.data), m2(pt2.data);
                m1 = geometry.block<3, 1>(3, 0);
                m2 = geometry.block<3, 1>(0,0);
                float sidelength = 0.15 * geometry(6);
                display->addSquare(pt1, pt2, sidelength, id, false, viewport_id);
                ++id;
                display->addSquare(pt1, pt2, sidelength, id, false, viewport_id + 1);
                ++id;
            } else {
                pcl::PointXYZ pt1, pt2;
                Eigen::Map<wave::Vec3f> m1(pt1.data), m2(pt2.data);

                float sidelength = 0.1 * geometry(6);

                m1 = geometry.block<3, 1>(3, 0) - sidelength*geometry.block<3, 1>(0,0);
                m2 = geometry.block<3, 1>(3, 0) + sidelength*geometry.block<3, 1>(0,0);
                display->addLine(pt1, pt2, id, id + 1, false, viewport_id);
                id += 2;
                display->addLine(pt1, pt2, id, id + 1, false, viewport_id + 1);
                id += 2;
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        *cloud1 = odom->undis_candidates_cur.at(i);
        *cloud2 = odom->undis_candidates_prev.at(i);
        display->addPointcloud(cloud1, ptcld_id, false, viewport_id);
        ++ptcld_id;
        display->addPointcloud(cloud2, ptcld_id, false, viewport_id + 1);
        ++ptcld_id;
    }
}

wave::TimeType parseTime(const std::string &date_time) {
    std::tm tm = {};
    unsigned long nano_seconds = std::stoi(date_time.substr(20, 9));
    std::chrono::nanoseconds n_secs(nano_seconds);
    std::stringstream ss(date_time);
    ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
    std::chrono::system_clock::time_point sys_clock = std::chrono::system_clock::from_time_t(std::mktime(&tm));
    wave::TimeType stamp;
    stamp = stamp + sys_clock.time_since_epoch() + n_secs;
    return stamp;
}

}

int main(int argc, char** argv) {
    if (argc != 3) {
        throw std::runtime_error("Must be run with only 2 arguments: \n 1. Full path to data \n 2. Full path to configuration files");
    }
    std::string data_path(argv[1]);
    std::string config_path(argv[2]);
    if(data_path.back() != '/') {
        data_path = data_path + '/';
    }
    if(config_path.back() != '/') {
        config_path = config_path + '/';
    }

    wave::ConfigParser parser;
    wave::MatX cutoff_angles;
    parser.addParam("cutoff_angles", &cutoff_angles);
    parser.load(config_path + "kitti_angles.yaml");

    wave::LaserOdomParams params;
    wave::FeatureExtractorParams feature_params;
    loadFeatureParams(config_path, "features.yaml", feature_params);
    setupFeatureParameters(feature_params);
    LoadParameters(config_path, "odom.yaml",  params);
    wave::TransformerParams transformer_params;
    transformer_params.traj_resolution = params.num_trajectory_states;

    wave::LaserOdom odom(params, feature_params, transformer_params);
    wave::PointCloudDisplay display("Kitti Eval"); //, 0.2, 3, 2);
    display.startSpin();

    auto func = [&]() {updateVisualizer(&odom, &display);};
    odom.registerOutputFunction(func);

    //set up pointcloud iterators
    boost::filesystem::path p(data_path + "velodyne_points/data");
    std::vector<boost::filesystem::path> v;
    std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
    std::sort(v.begin(), v.end());

    //timesteps
    fstream timestamps_start(data_path + "velodyne_points/timestamps_start.txt", ios::in);
    fstream timestamps_end(data_path + "velodyne_points/timestamps_end.txt", ios::in);

    if (!timestamps_start.good() || !timestamps_end.good()) {
        throw std::runtime_error("file structure not as expected for timestamps");
    }

    std::vector<wave::TimeType> time_start, time_end;
    std::string cur_start_stamp, cur_end_stamp;
    while (std::getline(timestamps_start, cur_start_stamp) && std::getline(timestamps_end, cur_end_stamp)) {
        time_start.emplace_back(parseTime(cur_start_stamp));
        time_end.emplace_back(parseTime(cur_end_stamp));
    }

    unsigned long counter = 0;
    pcl::PointCloud<pcl::PointXYZI> ptcloud;
    std::vector<double> bins(2000);
    std::vector<unsigned long> bincnt(2000);
    std::fill(bincnt.begin(), bincnt.end(), 0);
    double start = -26.0 * (M_PI / 180.0);
    std::generate(bins.begin(), bins.end(), [&start] () {
        start += M_PI / 10800.0;
        return start;
    });
    std::vector<std::vector<double>> azimuths(65);
    std::vector<std::vector<double>> elevations(65);
    std::vector<std::vector<double>> ranges(65);
    std::vector<std::vector<double>> xydistances(65);
    std::vector<std::vector<unsigned long>> pt_order(65);
    bool binary_format = false;
    unsigned long pt_idx = 0;
    unsigned long ring_index = 0;
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
        if (counter < 432)
            continue;
        auto diff = end_t - start_t;
        ptcloud.clear();
        azimuths.at(ring_index).clear();
        elevations.at(ring_index).clear();
        ranges.at(ring_index).clear();
        pt_order.at(ring_index).clear();
        pt_idx = 0;
        ring_index = 0;
        double prev_azimuth = 0;
        while (cloud_file.good() && !cloud_file.eof()) {
            pcl::PointXYZI pt;
            if (binary_format) {
                cloud_file.read((char *) pt.data, 3*sizeof(float));
                cloud_file.read((char *) &pt.intensity, sizeof(float));
            } else {
                std::string line;
                std::getline(cloud_file, line);
                std::stringstream ss(line);
                ss >> pt.x;
                ss >> pt.y;
                ss >> pt.z;
                ss >> pt.intensity;
            }

            auto elevation = std::atan2(pt.z, std::sqrt(pt.x * pt.x + pt.y * pt.y));
            auto range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            auto xydistance = std::sqrt(pt.x * pt.x + pt.y * pt.y);

            auto bnd = std::lower_bound(bins.begin(), bins.end(), elevation);
            bincnt.at((bnd - bins.begin()) - 1) += 1;

            auto ang = (std::atan2(pt.y, pt.x));
            ang < 0 ? ang = ang + 2.0*M_PI : ang;

            double from_start = ang - cutoff_angles(ring_index, 0);
            from_start < 0 ? from_start = from_start + 2.0*M_PI : from_start;

            if (prev_azimuth > ang + 0.2) {
                ++ring_index;
                azimuths.at(ring_index).clear();
                elevations.at(ring_index).clear();
                ranges.at(ring_index).clear();
                pt_order.at(ring_index).clear();
                xydistances.at(ring_index).clear();
            }
            prev_azimuth = ang;

            pt.intensity = (float) (from_start) * 100.0f;

//            const double ang_tol = 0;
//            if (ang > ang_tol && ang < (2*M_PI - ang_tol)) {
            azimuths.at(ring_index).emplace_back(ang);
            elevations.at(ring_index).emplace_back(elevation);
            ranges.at(ring_index).emplace_back(range);
            xydistances.at(ring_index).emplace_back(xydistance);
            pt_order.at(ring_index).emplace_back(pt_idx);
            ptcloud.push_back(pt);
//            }
            ++pt_idx;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr viz_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        *viz_cloud = ptcloud;
        display.addPointcloud(viz_cloud, 0);
        std::this_thread::sleep_for(diff);
        for (uint32_t i = 12; i < 17; ++i) {
            matplotlibcpp::subplot(2, 2, 1);
            matplotlibcpp::plot(pt_order.at(i), azimuths.at(i));
            matplotlibcpp::subplot(2,2,2);
            matplotlibcpp::plot(pt_order.at(i), elevations.at(i));
            matplotlibcpp::subplot(2,2,3);
            matplotlibcpp::plot(pt_order.at(i), ranges.at(i), ".");
            matplotlibcpp::subplot(2, 2, 4);
            matplotlibcpp::plot(azimuths.at(i), ranges.at(i), ".");
//            matplotlibcpp::title("Ring no. " + std::to_string(i));
        }
        matplotlibcpp::show(true);
    }
    matplotlibcpp::plot(bins, bincnt);
    matplotlibcpp::show(true);

    display.stopSpin();
    return 0;
}