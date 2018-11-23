#include <pcl/io/pcd_io.h>

#include "wave/wave_test.hpp"
#include "wave/utils/config.hpp"
#include "wave/odometry/feature_extractor.hpp"
#include "wave/odometry/PointXYZIR.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace wave {

namespace {

void loadParameters(const std::string &path, const std::string &filename, FeatureExtractorParams &params) {
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

void setupParameters(FeatureExtractorParams &param) {
    std::vector<Criteria> edge_high, edge_low, flat, edge_int_high, edge_int_low;
    edge_high.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::HIGH_POS, &(param.edge_tol)});

    edge_low.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::HIGH_NEG, &(param.edge_tol)});

    flat.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.flat_tol)});

    edge_int_high.emplace_back(
      Criteria{Signal::INTENSITY, Kernel::FOG, SelectionPolicy::HIGH_POS, &(param.int_edge_tol)});
    edge_int_high.emplace_back(
      Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_high.emplace_back(
      Criteria{Signal::RANGE, Kernel::VAR, SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    edge_int_low.emplace_back(
      Criteria{Signal::INTENSITY, Kernel::FOG, SelectionPolicy::HIGH_NEG, &(param.int_edge_tol)});
    edge_int_low.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_low.emplace_back(
      Criteria{Signal::RANGE, Kernel::VAR, SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    param.feature_definitions.clear();
    param.feature_definitions.emplace_back(FeatureDefinition{edge_high, &(param.n_edge)});
    param.feature_definitions.emplace_back(FeatureDefinition{edge_low, &(param.n_edge)});
    param.feature_definitions.emplace_back(FeatureDefinition{flat, &(param.n_flat)});
    param.feature_definitions.emplace_back(FeatureDefinition{edge_int_high, &(param.n_int_edge)});
    param.feature_definitions.emplace_back(FeatureDefinition{edge_int_low, &(param.n_int_edge)});
}
}

// Fixture to perform setup
class FeatureExtractorTests : public testing::Test {
 protected:
    FeatureExtractorTests() {}

    virtual ~FeatureExtractorTests() {}

    virtual void SetUp() {
        loadParameters("config/", "features.yaml", params);
        uint32_t rings = 32;
        setupParameters(params);
        extractor.setParams(params, rings);
    }

    void setupContainers() {
        this->scan.resize(32);
        this->signals.resize(32);
        this->range.resize(32);
        std::fill(this->range.begin(), this->range.end(), 0);
        this->indices.resize(this->params.N_FEATURES);
        for (uint32_t i = 0; i < this->params.N_FEATURES; i++) {
            this->indices.at(i).resize(32);
        }
    }

    void setupScan() {
        for (uint32_t i = 0; i < 32; i++) {
            this->scan.at(i) = Eigen::Tensor<float, 2>(5, 2200);
            this->signals.at(i).emplace(Signal::RANGE, Eigen::Tensor<float, 1>(2200));
            this->signals.at(i).emplace(Signal::INTENSITY, Eigen::Tensor<float, 1>(2200));
        }
    }

    void loadPCDfromDirectory(const std::string &directory) {
        boost::filesystem::path p(directory);
        std::vector<boost::filesystem::path> v;
        std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
        std::sort(v.begin(), v.end());
        pcl::io::loadPCDFile(v.front().string(), this->ref);

        for (auto pt : this->ref) {
            float ang = (float) (std::atan2(pt.y, pt.x) * -1.0);
            // need to fraction of complete rotation
            ang < 0 ? ang = ang + 2.0 * M_PI : ang;
            ang /= 2.0 * M_PI;

            float ts = ang * 0.1;

            float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (range < 3.0f) {
                continue;
            }

            this->scan.at(pt.ring)(0, this->range.at(pt.ring)) = pt.x;
            this->scan.at(pt.ring)(1, this->range.at(pt.ring)) = pt.y;
            this->scan.at(pt.ring)(2, this->range.at(pt.ring)) = pt.z;
            this->scan.at(pt.ring)(3, this->range.at(pt.ring)) = ts;
            this->scan.at(pt.ring)(4, this->range.at(pt.ring)) = ang;

            this->signals.at(pt.ring).at(Signal::RANGE)(this->range.at(pt.ring)) = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            this->signals.at(pt.ring).at(Signal::INTENSITY)(this->range.at(pt.ring)) = pt.intensity;
            this->range.at(pt.ring) += 1;
        }
    }

    pcl::PointCloud<PointXYZIR> ref;
    FeatureExtractorParams params;
    FeatureExtractor extractor;
    FeatureExtractor::Tensorf scan;
    FeatureExtractor::SignalVec signals;
    std::vector<int> range;
    FeatureExtractor::TensorIdx indices;
};

TEST(Feature_Extractor, Initialization) {
    FeatureExtractorParams params;
    loadParameters("config/", "features.yaml", params);
    uint32_t rings = 32;
    setupParameters(params);
    FeatureExtractor extractor(params, rings);
}

// Check that uninitialized extractor throws exception if it is used
TEST(Feature_Extractor, DefaultConstructor) {
    FeatureExtractor extractor;
    FeatureExtractor::Tensorf scan;
    FeatureExtractor::SignalVec signals;
    std::vector<int> range;
    FeatureExtractor::TensorIdx indices;
    EXPECT_THROW(extractor.getFeatures(scan, signals, range, indices), std::length_error);
}

TEST_F(FeatureExtractorTests, DelayedInit) {}

// Given scan, signal, and range objects that are not formatted as expected, should report the error as exception
TEST_F(FeatureExtractorTests, UnformatedStructures) {
    EXPECT_THROW(this->extractor.getFeatures(this->scan, this->signals, this->range, this->indices), std::length_error);
}

// Given scan, signal, and range objects that are formatted properly, but are empty, should not throw
TEST_F(FeatureExtractorTests, NoData) {
    this->setupContainers();
    EXPECT_NO_THROW(this->extractor.getFeatures(this->scan, this->signals, this->range, this->indices));
}

// Given an actual scan, run feature extraction
TEST_F(FeatureExtractorTests, ProcessScan) {
    this->setupContainers();
    this->setupScan();
    this->loadPCDfromDirectory("/home/ben/rosbags/last_ditch_bags/pcd");

    EXPECT_NO_THROW(extractor.getFeatures(this->scan, this->signals, this->range, this->indices));
}

TEST_F(FeatureExtractorTests, VisualizePrefiltering) {
    this->setupContainers();
    this->setupScan();
    this->loadPCDfromDirectory("/home/ben/rosbags/last_ditch_bags/pcd");

    extractor.preFilter(this->scan, this->signals, this->range);
    pcl::PointCloud<pcl::PointXYZ> full_scan;
    pcl::PointCloud<pcl::PointXYZI> filtered_scan;

    for (uint32_t ring_id = 0; ring_id < extractor.valid_pts.size(); ++ring_id) {
        for (uint32_t pt_idx = 0; pt_idx < range.at(ring_id); ++pt_idx) {
            pcl::PointXYZ cur_point;
            pcl::PointXYZI cur_point_i;
            cur_point_i.x = cur_point.x = this->scan.at(ring_id)(0, pt_idx);
            cur_point_i.y = cur_point.y = this->scan.at(ring_id)(1, pt_idx);
            cur_point_i.z = cur_point.z = this->scan.at(ring_id)(2, pt_idx);
            full_scan.push_back(cur_point);
            if (extractor.valid_pts.at(ring_id)(pt_idx)) {
                cur_point_i.intensity = 1;
            } else {
                cur_point_i.intensity = 0;
            }
            filtered_scan.push_back(cur_point_i);
        }
    }

    PointCloudDisplay display("filtered_points", 0.2, 2, 1);

    display.addPointcloud(full_scan.makeShared(), 0, false, 1);
    display.addPointcloud(filtered_scan.makeShared(), 1, false, 2);
    display.startSpin();
    cin.get();
    display.stopSpin();
}

TEST_F(FeatureExtractorTests, VisualizeFeatures) {
    this->setupContainers();
    this->setupScan();
    this->loadPCDfromDirectory("/home/ben/rosbags/last_ditch_bags/pcd");

    extractor.getFeatures(this->scan, this->signals, this->range, this->indices);

    pcl::PointCloud<pcl::PointXYZ> ref_cloud;
    pcl::copyPointCloud(this->ref, ref_cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>> feature_viz;
    feature_viz.resize(3);

    PointCloudDisplay display("LIDAR Features", 1, 2, 2);
    display.startSpin();
    display.addPointcloud(ref_cloud.makeShared(), 10, false, 4);

    for (uint32_t i = 0; i < 3; i++) {
        feature_viz.at(i).clear();
        for (uint32_t j = 0; j < 32; j++) {
            for (int k = 0; k < this->indices.at(i).at(j).dimension(0); k++) {
                pcl::PointXYZ pt;
                pt.x = this->scan.at(j)(0, this->indices.at(i).at(j)(k));
                pt.y = this->scan.at(j)(1, this->indices.at(i).at(j)(k));
                pt.z = this->scan.at(j)(2, this->indices.at(i).at(j)(k));

                feature_viz.at(i).push_back(pt);
            }
        }
        display.addPointcloud(feature_viz.at(i).makeShared(), i, false, i+1);
    }

    cin.get();
}

TEST_F(FeatureExtractorTests, VisualizeMooseFeatures) {
    this->setupContainers();
    this->setupScan();

    fstream cloud_file;
    cloud_file.open("/home/ben/rosbags/wat_27/lidar/0000002000.bin", ios::in | ios::binary);

    if (!cloud_file.good()) {
        throw std::runtime_error("Cannot find pointcloud file");
    }

    pcl::PointCloud<pcl::PointXYZI> display_cloud;
    while (cloud_file.good() && !cloud_file.eof()) {
        pcl::PointXYZI pt;
        uint16_t ring;
        int32_t nanosec_offset;
        float range, azimuth, elevation, raw_intensity;

        cloud_file.read((char *) pt.data, 3 * sizeof(float));
        cloud_file.read((char *) &raw_intensity, sizeof(float));
        cloud_file.read((char *) &range, sizeof(float));
        cloud_file.read((char *) &azimuth, sizeof(float));
        cloud_file.read((char *) &elevation, sizeof(float));
        cloud_file.read((char *) &(ring), sizeof(uint16_t));
        cloud_file.read((char *) &nanosec_offset, sizeof(int32_t));

        pt.intensity = raw_intensity;

        display_cloud.push_back(pt);

        if (std::isnan(pt.x)) {
            continue;
        }

        if (range < 3.0f) {
            continue;
        }

        typedef std::chrono::duration<float, std::nano> float_nanos;
        typedef std::chrono::duration<float> float_seconds;
        float_seconds pt_time;
        pt_time = std::chrono::duration_cast<float_seconds>(std::chrono::duration_cast<float_nanos>(std::chrono::nanoseconds(nanosec_offset)));

        this->scan.at(ring)(0, this->range.at(ring)) = pt.x;
        this->scan.at(ring)(1, this->range.at(ring)) = pt.y;
        this->scan.at(ring)(2, this->range.at(ring)) = pt.z;
        this->scan.at(ring)(3, this->range.at(ring)) = pt_time.count();
        this->scan.at(ring)(4, this->range.at(ring)) = azimuth;

        this->signals.at(ring).at(Signal::RANGE)(this->range.at(ring)) = range;
        this->signals.at(ring).at(Signal::INTENSITY)(this->range.at(ring)) = pt.intensity;
        this->range.at(ring) += 1;
    }

    extractor.getFeatures(this->scan, this->signals, this->range, this->indices);

    PointCloudDisplay display("VLP-32C", 1, 2, 1);
    display.startSpin();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::vector<pcl::PointCloud<pcl::PointXYZ>> feature_viz;
    feature_viz.resize(3);

    pcl::PointCloud<pcl::PointXYZ> display_viz;
    pcl::copyPointCloud(display_cloud, display_viz);

    display.addPointcloud(display_viz.makeShared(), 0, false, 1);

    for (uint32_t i = 0; i < 3; i++) {
        feature_viz.at(i).clear();
        for (uint32_t j = 0; j < 32; j++) {
            for (int k = 0; k < this->indices.at(i).at(j).dimension(0); k++) {
                pcl::PointXYZ pt;
                pt.x = this->scan.at(j)(0, this->indices.at(i).at(j)(k));
                pt.y = this->scan.at(j)(1, this->indices.at(i).at(j)(k));
                pt.z = this->scan.at(j)(2, this->indices.at(i).at(j)(k));

                feature_viz.at(i).push_back(pt);
            }
        }

        display.addPointcloud(feature_viz.at(i).makeShared(), i + 1, false, 2);
    }

    cin.get();
    display.stopSpin();

    cloud_file.close();
}
}
