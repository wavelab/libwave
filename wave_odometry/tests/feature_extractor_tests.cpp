#include <pcl/io/pcd_io.h>

#include "wave/wave_test.hpp"
#include "wave/utils/config.hpp"
#include "wave/odometry/feature_extractor.hpp"
#include "wave/odometry/PointXYZIR.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace wave{

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
    std::vector<FeatureExtractorParams::Criteria> edge_high, edge_low, flat, edge_int_high, edge_int_low;
    edge_high.emplace_back(FeatureExtractorParams::Criteria{FeatureExtractorParams::Signal::RANGE,
                                                            FeatureExtractorParams::Kernel::LOAM,
                                                            FeatureExtractorParams::SelectionPolicy::HIGH_POS,
                                                            &(param.edge_tol)});

    edge_low.emplace_back(FeatureExtractorParams::Criteria{FeatureExtractorParams::Signal::RANGE,
                                                            FeatureExtractorParams::Kernel::LOAM,
                                                            FeatureExtractorParams::SelectionPolicy::HIGH_NEG,
                                                            &(param.edge_tol)});

    flat.emplace_back(FeatureExtractorParams::Criteria{FeatureExtractorParams::Signal::RANGE,
                                                           FeatureExtractorParams::Kernel::LOAM,
                                                           FeatureExtractorParams::SelectionPolicy::NEAR_ZERO,
                                                           &(param.flat_tol)});

    edge_int_high.emplace_back(FeatureExtractorParams::Criteria{FeatureExtractorParams::Signal::INTENSITY,
                                                       FeatureExtractorParams::Kernel::FOG,
                                                       FeatureExtractorParams::SelectionPolicy::HIGH_POS,
                                                       &(param.int_edge_tol)});
    edge_int_high.emplace_back(FeatureExtractorParams::Criteria{FeatureExtractorParams::Signal::RANGE,
                                                                FeatureExtractorParams::Kernel::LOAM,
                                                                FeatureExtractorParams::SelectionPolicy::NEAR_ZERO,
                                                                &(param.int_flat_tol)});
    edge_int_high.emplace_back(FeatureExtractorParams::Criteria{FeatureExtractorParams::Signal::RANGE,
                                                                FeatureExtractorParams::Kernel::RNG_VAR,
                                                                FeatureExtractorParams::SelectionPolicy::NEAR_ZERO,
                                                                &(param.variance_limit_rng)});

    edge_int_low.emplace_back(FeatureExtractorParams::Criteria{FeatureExtractorParams::Signal::INTENSITY,
                                                                FeatureExtractorParams::Kernel::FOG,
                                                                FeatureExtractorParams::SelectionPolicy::HIGH_NEG,
                                                                &(param.int_edge_tol)});
    edge_int_low.emplace_back(FeatureExtractorParams::Criteria{FeatureExtractorParams::Signal::RANGE,
                                                                FeatureExtractorParams::Kernel::LOAM,
                                                                FeatureExtractorParams::SelectionPolicy::NEAR_ZERO,
                                                                &(param.int_flat_tol)});
    edge_int_low.emplace_back(FeatureExtractorParams::Criteria{FeatureExtractorParams::Signal::RANGE,
                                                                FeatureExtractorParams::Kernel::RNG_VAR,
                                                                FeatureExtractorParams::SelectionPolicy::NEAR_ZERO,
                                                                &(param.variance_limit_rng)});

    param.feature_definitions.clear();
    param.feature_definitions.emplace_back(FeatureExtractorParams::FeatureDefinition{edge_high, &(param.n_edge)});
    param.feature_definitions.emplace_back(FeatureExtractorParams::FeatureDefinition{edge_low, &(param.n_edge)});
    param.feature_definitions.emplace_back(FeatureExtractorParams::FeatureDefinition{flat, &(param.n_flat)});
    param.feature_definitions.emplace_back(FeatureExtractorParams::FeatureDefinition{edge_int_high, &(param.n_int_edge)});
    param.feature_definitions.emplace_back(FeatureExtractorParams::FeatureDefinition{edge_int_low, &(param.n_int_edge)});
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

    FeatureExtractorParams params;
    FeatureExtractor extractor;
    FeatureExtractor::Tensorf scan, signals;
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
    FeatureExtractor::Tensorf scan, signals;
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
    this->scan.resize(32);
    this->signals.resize(32);
    this->range.resize(32);
    std::fill(this->range.begin(), this->range.end(), 0);
    this->indices.resize(this->params.N_FEATURES);
    for(uint32_t i = 0; i < this->params.N_FEATURES; i++) {
        this->indices.at(i).resize(32);
    }
    EXPECT_NO_THROW(this->extractor.getFeatures(this->scan, this->signals, this->range, this->indices));
}

// Given an actual scan, run feature extraction
TEST_F(FeatureExtractorTests, ProcessScan) {
    this->scan.resize(32);
    this->signals.resize(32);
    this->range.resize(32);
    std::fill(this->range.begin(), this->range.end(), 0);
    for(uint32_t i = 0; i < 32; i++) {
        this->scan.at(i) = Eigen::Tensor<float, 2>(5, 2200);
        this->signals.at(i) = Eigen::Tensor<float, 2>(2, 2200);
    }

    this->indices.resize(this->params.N_FEATURES);
    for(uint32_t i = 0; i < this->params.N_FEATURES; i++) {
        this->indices.at(i).resize(32);
    }

    pcl::PointCloud<PointXYZIR> ref;
    pcl::io::loadPCDFile("data/testscan.pcd", ref);

    for(auto pt : ref) {
        float ang =(float) (std::atan2(pt.y, pt.x) * -1.0);
        // need to fraction of complete rotation
        ang < 0 ? ang = ang + 2.0*M_PI : ang;
        ang /= 2.0*M_PI;

        this->scan.at(pt.ring)(0, this->range.at(pt.ring)) = pt.x;
        this->scan.at(pt.ring)(1, this->range.at(pt.ring)) = pt.y;
        this->scan.at(pt.ring)(2, this->range.at(pt.ring)) = pt.z;
        this->scan.at(pt.ring)(3, this->range.at(pt.ring)) = ang;
        this->scan.at(pt.ring)(4, this->range.at(pt.ring)) = 0;

        this->signals.at(pt.ring)(0, this->range.at(pt.ring)) = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        this->signals.at(pt.ring)(1, this->range.at(pt.ring)) = pt.intensity;
        this->range.at(pt.ring) += 1;
    }

    EXPECT_NO_THROW(extractor.getFeatures(this->scan, this->signals, this->range, this->indices));
}

TEST_F(FeatureExtractorTests, VisualizeFeatures) {
    this->scan.resize(32);
    this->signals.resize(32);
    this->range.resize(32);
    std::fill(this->range.begin(), this->range.end(), 0);
    for(uint32_t i = 0; i < 32; i++) {
        this->scan.at(i) = Eigen::Tensor<float, 2>(5, 2200);
        this->signals.at(i) = Eigen::Tensor<float, 2>(2, 2200);
    }

    this->indices.resize(this->params.N_FEATURES);
    for(uint32_t i = 0; i < this->params.N_FEATURES; i++) {
        this->indices.at(i).resize(32);
    }

    pcl::PointCloud<PointXYZIR> ref;
    pcl::io::loadPCDFile("data/testscan.pcd", ref);

    for(auto pt : ref) {
        float ang =(float) (std::atan2(pt.y, pt.x) * -1.0);
        // need to fraction of complete rotation
        ang < 0 ? ang = ang + 2.0*M_PI : ang;
        ang /= 2.0*M_PI;

        this->scan.at(pt.ring)(0, this->range.at(pt.ring)) = pt.x;
        this->scan.at(pt.ring)(1, this->range.at(pt.ring)) = pt.y;
        this->scan.at(pt.ring)(2, this->range.at(pt.ring)) = pt.z;
        this->scan.at(pt.ring)(3, this->range.at(pt.ring)) = ang;
        this->scan.at(pt.ring)(4, this->range.at(pt.ring)) = 0;

        this->signals.at(pt.ring)(0, this->range.at(pt.ring)) = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        this->signals.at(pt.ring)(1, this->range.at(pt.ring)) = pt.intensity;
        this->range.at(pt.ring) += 1;
    }

    extractor.getFeatures(this->scan, this->signals, this->range, this->indices);

    pcl::PointCloud<pcl::PointXYZ> ref_cloud;
    pcl::copyPointCloud(ref, ref_cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>> feature_viz;
    feature_viz.resize(3);

    PointCloudDisplay display("LIDAR Features");
    display.startSpin();

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
        display.addPointcloud(feature_viz.at(i).makeShared(), i);
    }

    cin.get();
}

}
