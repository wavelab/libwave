#include "wave/wave_test.hpp"
#include "wave/odometry/feature_extractor.hpp"
#include "wave/odometry/line_fitter.hpp"
#include "../data/include/config_utils.hpp"
#include "../../kitti_tools/include/kitti_utility_methods.hpp"

namespace wave {

// Fixture to perform setup
class LineFitterTests : public testing::Test {
 protected:
    LineFitterTests() {}

    virtual ~LineFitterTests() {}

    virtual void SetUp() {
        loadFeatureParams("config/", "features.yaml", params);
        uint32_t rings = 32;
        setupFeatureParameters(params);
        extractor.setParams(params, rings);
    }

    FeatureExtractorParams params;
    FeatureExtractor extractor;
    FeatureExtractor::Tensorf scan;
    FeatureExtractor::SignalVec signals;
    std::vector<int> range;
    FeatureExtractor::TensorIdx indices;
};

TEST_F(LineFitterTests, Constructor) {
    LineFitter line_fitter(1.5, 8, 0.1, 4);
}

TEST_F(LineFitterTests, FitLines) {
    // Run feature extractor to prepare candidate cloud
    this->scan.resize(32);
    this->signals.resize(32);
    this->range.resize(32);
    std::fill(this->range.begin(), this->range.end(), 0);
    for (uint32_t i = 0; i < 32; i++) {
        this->scan.at(i) = Eigen::Tensor<float, 2>(5, 2200);
        this->signals.at(i).emplace(Signal::RANGE, Eigen::Tensor<float, 1>(2200));
        this->signals.at(i).emplace(Signal::INTENSITY, Eigen::Tensor<float, 1>(2200));
    }

    this->indices.resize(this->params.N_FEATURES);
    for (uint32_t i = 0; i < this->params.N_FEATURES; i++) {
        this->indices.at(i).resize(32);
    }

    pcl::PointCloud<PointXYZIR> ref;
    boost::filesystem::path p("/home/ben/rosbags/last_ditch_bags/pcd");
    std::vector<boost::filesystem::path> v;
    std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
    std::sort(v.begin(), v.end());
    pcl::io::loadPCDFile(v.front().string(), ref);

    for (auto pt : ref) {
        float ang = (float) (std::atan2(pt.y, pt.x) * -1.0);
        // need to fraction of complete rotation
        ang < 0 ? ang = ang + 2.0 * M_PI : ang;
        ang /= 20 * M_PI;

        this->scan.at(pt.ring)(0, this->range.at(pt.ring)) = pt.x;
        this->scan.at(pt.ring)(1, this->range.at(pt.ring)) = pt.y;
        this->scan.at(pt.ring)(2, this->range.at(pt.ring)) = pt.z;
        this->scan.at(pt.ring)(3, this->range.at(pt.ring)) = ang;
        this->scan.at(pt.ring)(4, this->range.at(pt.ring)) = 0;

        this->signals.at(pt.ring).at(Signal::RANGE)(this->range.at(pt.ring)) = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        this->signals.at(pt.ring).at(Signal::INTENSITY)(this->range.at(pt.ring)) = pt.intensity;
        this->range.at(pt.ring) += 1;
    }

    this->extractor.getFeatures(this->scan, this->signals, this->range, this->indices);

    // Now create pointcloud of line candidates for visualization & for line fitting
    pcl::PointCloud<pcl::PointXYZ> candidates;
    MatXf candidate_cloud;

    int pointcount = 0;
    for (uint32_t ring_idx = 0; ring_idx < 32; ++ring_idx) {
        pointcount += this->indices.at(0).at(ring_idx).dimension(0);
    }
    candidate_cloud.resize(3,pointcount);

    int counter = 0;
    for (uint32_t ring_idx = 0; ring_idx < 32; ++ring_idx) {
        for (uint32_t i = 0; i < this->indices.at(0).at(ring_idx).dimension(0); ++i) {
            pcl::PointXYZ pt;
            auto idx = this->indices.at(0).at(ring_idx)(i);
            pt.x = this->scan.at(ring_idx)(0, idx);
            pt.y = this->scan.at(ring_idx)(1, idx);
            pt.z = this->scan.at(ring_idx)(2, idx);
            if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z < 9.0) {
                continue;
            }
            candidate_cloud(0, counter) = pt.x;
            candidate_cloud(1, counter) = pt.y;
            candidate_cloud(2, counter) = pt.z;
            counter++;
            candidates.push_back(pt);
        }
    }

    PointCloudDisplay display("Line Extractor");
    display.startSpin();
    display.addPointcloud(candidates.makeShared(), 0);

    LineFitter line_fitter(5, 10, 0.05, 4);
    line_fitter.fitLines(candidate_cloud);

    auto tracks = line_fitter.getTracks();

    std::cout << tracks.size() << "\n";

    int id = 10;
    for (const auto &track : tracks) {
        pcl::PointCloud<pcl::PointXYZ> feature_points;
        for (const auto &map : track.mapping) {
            pcl::PointXYZ new_pt;
            new_pt.x = candidates[map.pt_idx].x;
            new_pt.y = candidates[map.pt_idx].y;
            new_pt.z = candidates[map.pt_idx].z;
            feature_points.push_back(new_pt);
        }
        pcl::PointXYZ pt1, pt2;
        Eigen::Map<wave::Vec3f> m1(pt1.data), m2(pt2.data);

        double sidelength = 0.1 * track.mapping.size();

        m1 =
                (track.geometry.block<3, 1>(3, 0) - sidelength * track.geometry.block<3, 1>(0, 0)).cast<float>();
        m2 =
                (track.geometry.block<3, 1>(3, 0) + sidelength * track.geometry.block<3, 1>(0, 0)).cast<float>();
        display.addLine(pt1, pt2, id, id + 1);
        id += 2;

        display.addPointcloud(feature_points.makeShared(), id);
        id++;
    }

    cin.get();
}

}
