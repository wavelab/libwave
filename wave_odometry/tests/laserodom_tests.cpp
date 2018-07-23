#include <pcl/io/pcd_io.h>
#include <chrono>
#include <string>
#include "wave/wave_test.hpp"
#include "wave/utils/log.hpp"
#include "wave/utils/config.hpp"
#include "wave/odometry/laser_odom.hpp"
#include "wave/odometry/PointXYZIT.hpp"
#include "wave/containers/measurement_container.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include <boost/filesystem.hpp>

#define EIGEN_NO_MALLOC 1

namespace wave {

namespace {

const std::string TEST_SCAN = "data/testscan.pcd";
const std::string TEST_SEQUENCE_DIR = "data/garage/";

// Fixture to load same pointcloud all the time
class OdomTestFile : public testing::Test {
 protected:
    OdomTestFile() {}

    virtual ~OdomTestFile() {}

    virtual void SetUp() {
        pcl::io::loadPCDFile(TEST_SCAN, (this->ref));
        // filter out points on the car
        for (auto iter = this->ref.begin(); iter < this->ref.end(); iter++) {
            if ((iter->x > -3) && (iter->x < 2.5) && (iter->y > -1) && (iter->y < 1)) {
                this->ref.erase(iter);
            }
        }
    }

    pcl::PointCloud<PointXYZIR> ref;
};

void LoadSensorParameters(const std::string &path, const std::string &filename, RangeSensorParams &senparams) {
    wave::ConfigParser parser;
    parser.addParam("rings", &(senparams.rings));
    parser.addParam("sigma_spherical", &(senparams.sigma_spherical));
    parser.addParam("elevation_angles", &(senparams.elevation_angles));

    parser.load(path + filename);
}

void LoadParameters(const std::string &path, const std::string &filename, LaserOdomParams &params) {
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
    parser.addParam("scan_period", &(params.scan_period));
    parser.addParam("max_ticks", &(params.max_ticks));
    parser.addParam("n_ring", &(params.n_ring));

    LoadSensorParameters(path, "sensor_model.yaml", params.sensor_params);

    parser.addParam("azimuth_tol", &(params.azimuth_tol));
    parser.addParam("TTL", &(params.TTL));
    parser.addParam("min_eigen", &(params.min_eigen));
    parser.addParam("max_extrapolation", &(params.max_extrapolation));
    parser.addParam("output_trajectory", &(params.output_trajectory));
    parser.addParam("output_correspondences", &(params.output_correspondences));
    parser.addParam("only_extract_features", &(params.only_extract_features));
    parser.addParam("use_weighting", &(params.use_weighting));
    parser.addParam("plot_stuff", &(params.plot_stuff));
    parser.addParam("motion_prior", &(params.motion_prior));
    parser.addParam("no_extrapolation", &(params.no_extrapolation));
    parser.addParam("treat_lines_as_planes", &(params.treat_lines_as_planes));

    parser.load(path + filename);
}

void setupFeatureParameters(FeatureExtractorParams &param) {
    std::vector<Criteria> edge_high, edge_low, flat, edge_int_high, edge_int_low;
    edge_high.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::HIGH_POS, &(param.edge_tol)});

    edge_low.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::HIGH_NEG, &(param.edge_tol)});

    flat.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.flat_tol)});

    edge_int_high.emplace_back(
            Criteria{Signal::INTENSITY, Kernel::FOG, SelectionPolicy::HIGH_POS, &(param.int_edge_tol)});
    edge_int_high.emplace_back(
            Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_high.emplace_back(
            Criteria{Signal::RANGE, Kernel::RNG_VAR, SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    edge_int_low.emplace_back(
            Criteria{Signal::INTENSITY, Kernel::FOG, SelectionPolicy::HIGH_NEG, &(param.int_edge_tol)});
    edge_int_low.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_low.emplace_back(
            Criteria{Signal::RANGE, Kernel::RNG_VAR, SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    param.feature_definitions.clear();
    param.feature_definitions.emplace_back(FeatureDefinition{edge_high, &(param.n_edge)});
    param.feature_definitions.emplace_back(FeatureDefinition{edge_low, &(param.n_edge)});
    param.feature_definitions.emplace_back(FeatureDefinition{flat, &(param.n_flat)});
    param.feature_definitions.emplace_back(FeatureDefinition{edge_int_high, &(param.n_int_edge)});
    param.feature_definitions.emplace_back(FeatureDefinition{edge_int_low, &(param.n_int_edge)});
}

}

TEST(laserodom, Init) {
    LaserOdom odom(LaserOdomParams);
}

// See if packing data into a float works as I expect
TEST(Packing_test, intsintofloat) {
    uint16_t angle = 31226;
    uint8_t intensity = 134;
    float packed = 0;
    uint8_t *dest = static_cast<uint8_t *>(static_cast<void *>(&packed));
    memcpy(dest, &angle, 2);
    memcpy(dest + 2, &intensity, 1);
    uint16_t recovered_angle = *static_cast<uint16_t *>(static_cast<void *>(dest));
    uint8_t recovered_in = *static_cast<uint8_t *>(static_cast<void *>(dest + 2));
    ASSERT_EQ(angle, recovered_angle);
    ASSERT_EQ(intensity, recovered_in);
}

void updateVisualizer(const LaserOdom *odom, PointCloudDisplay *display) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr viz_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    *viz_cloud = odom->undistorted_cld;
    display->addPointcloud(viz_cloud, 0, true);
}

// This test is for odometry for the car moving in a straight line through the garage
TEST(OdomTest, StraightLineGarage) {
    // Load entire sequence into memory
    std::vector<pcl::PointCloud<PointXYZIR>> clds;
    std::vector<pcl::PointCloud<PointXYZIR>::Ptr> cldptr;
    pcl::PCLPointCloud2 temp;
    pcl::PointCloud<PointXYZIR> temp2;
    LOG_INFO("Starting to load clouds");
    boost::filesystem::path p("/home/bapskiko/rosbags/last_ditch_bags/pcd");
    std::vector<boost::filesystem::path> v;
    std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
    std::sort(v.begin(), v.end());
    int length = 0;
    int max_clouds;
    wave::ConfigParser parser;
    parser.addParam("max_clouds", &max_clouds);
    parser.load("config/test_length.yaml");
    for (auto iter = v.begin(); iter != v.end(); ++iter) {
        std::cout << "Loading cloud " << length << std::endl;
        pcl::io::loadPCDFile(iter->string(), temp);
        pcl::fromPCLPointCloud2(temp, temp2);
        clds.push_back(temp2);
        pcl::PointCloud<PointXYZIR>::Ptr ptr(new pcl::PointCloud<PointXYZIR>);
        *ptr = clds.at(length);
        cldptr.push_back(ptr);
        length++;
        if (length == max_clouds) {
            break;
        }
    }

    LOG_INFO("Finished loading clouds");
    // odom setup
    LaserOdomParams params;
    FeatureExtractorParams feature_params;
    setupFeatureParameters(feature_params);

    LoadParameters("config/", "odom.yaml",  params);

    TransformerParams transformer_params;
    transformer_params.traj_resolution = params.num_trajectory_states;

    LaserOdom odom(params, feature_params, transformer_params);
    std::vector<PointXYZIR> vec;
    uint16_t prev_enc = 0;
    uint16_t encoder = 0;

    PointCloudDisplay display("odom");
    display.startSpin();
    std::function<void()> func = std::bind(updateVisualizer, &odom, &display);
    odom.registerOutputFunction(func);

    // Loop through pointclouds and send points grouped by encoder angle odom
    TimeType start;
    auto offset = std::chrono::microseconds(0);
    for (int i = 0; i < length; i++) {
        std::cout << "Processing cloud " << i << " of " << length << "\n";
        for (const auto &pt : clds.at(i)) {
            PointXYZIR recovered;
            // unpackage intensity and encoder
            auto ang = (std::atan2(pt.y, pt.x) * -1.0);
            // need to convert to 0 to 2pi
            ang < 0 ? ang = ang + 2.0*M_PI : ang;

            encoder = (uint16_t) ((ang / (2.0*M_PI)) * 36000.0);

            // copy remaining fields
            recovered.x = pt.x;
            recovered.y = pt.y;
            recovered.z = pt.z;
            recovered.ring = pt.ring;
            recovered.intensity = pt.intensity;

            if (200 + encoder < prev_enc) {
                offset = offset + std::chrono::milliseconds(100);
            }

            if (encoder != prev_enc) {
                if (vec.size() > 0) {
                    std::chrono::microseconds dur((25 * encoder)/9);
                    TimeType stamp = start + offset + dur;
                    odom.addPoints(vec, prev_enc, stamp);
                    vec.clear();
                }
                prev_enc = encoder;
            }
            vec.emplace_back(recovered);
        }
//        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    auto end = std::chrono::steady_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms.\n";
}

TEST(Transforms, forward_backwards) {
    double base_rot[3] = {0.2, 0.1, -0.2};
    double base_trans[3] = {0.2, 5, 4};
    double pt[3] = {1, 2, -4};
    double scale = 0.33;

    // pt will be the point to change frames of
    // pt_start = R_scaled * pt + T
    double angle_axis[3] = {scale * base_rot[0], scale * base_rot[1], scale * base_rot[2]};
    double pt_start[3];
    ceres::AngleAxisRotatePoint(angle_axis, pt, pt_start);
    pt_start[0] += scale * base_trans[0];
    pt_start[1] += scale * base_trans[1];
    pt_start[2] += scale * base_trans[2];

    // pt_end = R_inv*pt_start - R_inv*T
    double angle_axis_inverse[3] = {-base_rot[0], -base_rot[1], -base_rot[2]};
    double pt_end[3], offset[3];
    ceres::AngleAxisRotatePoint(angle_axis_inverse, pt_start, pt_end);
    ceres::AngleAxisRotatePoint(angle_axis_inverse, base_trans, offset);
    pt_end[0] -= offset[0];
    pt_end[1] -= offset[1];
    pt_end[2] -= offset[2];

    // This should be a shortcut
    // pt_end = R_inv*R_scaled*pt + R_inv*T_scaled - R_inv * T
    // = R_(1-scaled)_inv*pt - (1 - scale)R_inv*T

    double rv_scale = 1 - scale;
    double angle_axis_scaled_inverse[3] = {
      -(rv_scale * base_rot[0]), -(rv_scale * base_rot[1]), -(rv_scale * base_rot[2])};
    double pt_end_simple[3];
    ceres::AngleAxisRotatePoint(angle_axis_scaled_inverse, pt, pt_end_simple);
    pt_end_simple[0] -= rv_scale * offset[0];
    pt_end_simple[1] -= rv_scale * offset[1];
    pt_end_simple[2] -= rv_scale * offset[2];

    ASSERT_NEAR(pt_end[0], pt_end_simple[0], 1e-6);
    ASSERT_NEAR(pt_end[1], pt_end_simple[1], 1e-6);
    ASSERT_NEAR(pt_end[2], pt_end_simple[2], 1e-6);
}

}  // namespace wave
