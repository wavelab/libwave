#include <pcl/io/pcd_io.h>
#include <chrono>
#include <pcap.h>
#include <string>
#include "wave/wave_test.hpp"
#include "wave/utils/log.hpp"
#include "wave/odometry/LaserOdom.hpp"
#include "wave/odometry/PointXYZIT.hpp"
#include "wave/odometry/laser_odom_residuals.hpp"
#include "wave/containers/measurement_container.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include <boost/filesystem.hpp>

namespace wave {

const std::string TEST_SCAN = "data/testscan.pcd";
const std::string TEST_SEQUENCE_DIR = "data/garage/";
const int sequence_length = 80;

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

TEST(laserodom, Init) {
    LaserOdom odom(LaserOdomParams());
}
// This visualizes the sequence in order to check that my hacky packaging works
// Should produce visualization coloured by encoder angle

TEST(laserodom, VizSequence) {
    // Playback in visualizer
    PointCloudDisplay display("sequence");
    display.startSpin();
    // Put entire sequence in memory
    std::vector<pcl::PointCloud<pcl::PointXYZI>> clds;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cldptr;
    pcl::PointCloud<pcl::PointXYZI> temp;
    for (int i = 0; i < sequence_length; i++) {
        pcl::io::loadPCDFile(TEST_SEQUENCE_DIR + std::to_string(i) + ".pcd", temp);
        clds.push_back(temp);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZI>);

        for (size_t j = 0; j < clds.at(i).size(); j++) {
            float packed = clds.at(i).at(j).intensity;
            uint16_t encoder = *static_cast<uint16_t *>(static_cast<void *>(&packed));
            clds.at(i).at(j).intensity = static_cast<float>(encoder);
        }
        *ptr = clds.at(i);
        cldptr.push_back(ptr);
    }

    for (int i = 0; i < sequence_length; i++) {
        display.addPointcloud(cldptr.at(i), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    display.stopSpin();
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

TEST(Residual_test, pointToLineAnalytic) {
    double base_rot[3] = {0.5, 0.3, -0.2};
    double base_trans[3] = {0.2, 5, 4};
    const double **trans, **rot;
    trans = new const double *;
    rot = new const double *;
    rot[0] = new const double[3]{base_rot[0], base_rot[1], base_rot[2]};
    trans[0] = new const double[3]{base_trans[0], base_trans[1], base_trans[2]};

    const double **composed;
    composed = new const double *[2];
    composed[0] = rot[0];
    composed[1] = trans[0];

    double eps = 1.5e-8;
    const double **trans_perturbed[6];
    const double **rot_perturbed[6];
    for (int i = 0; i < 3; i++) {
        trans_perturbed[i] = new const double *;
        base_trans[i] += eps;
        trans_perturbed[i][0] = new const double[3]{base_trans[0], base_trans[1], base_trans[2]};
        base_trans[i] -= eps;
    }
    for (int i = 0; i < 3; i++) {
        rot_perturbed[i] = new const double *;
        base_rot[i] += eps;
        rot_perturbed[i][0] = new const double[3]{base_rot[0], base_rot[1], base_rot[2]};
        base_rot[i] -= eps;
    }

    double **jacobian;
    jacobian = new double *[2];
    jacobian[0] = new double[9];
    jacobian[1] = new double[9];
    double **jacobian1;
    jacobian1 = new double *[3];
    jacobian1[0] = new double[6];
    jacobian1[1] = new double[6];
    jacobian1[2] = new double[6];

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, 0};
    double pt[3] = {1, 2, -4};
    double scale = 1;
    double residual[3] = {0};
    double residual1[3] = {0};

    AnalyticalPointToLine thing(pt, ptA, ptB, &scale);

    thing.Evaluate(composed, residual, jacobian);

    // Now check numerically here as well.
    for (int i = 0; i < 3; i++) {
        const double **compd;
        compd = new const double *[2];
        compd[0] = rot_perturbed[i][0];
        compd[1] = trans[0];

        thing.Evaluate(compd, residual1, jacobian);
        delete compd;

        for (int r = 0; r < 3; r++) {
            jacobian1[r][i] = (residual1[r] - residual[r]) / eps;
            EXPECT_NEAR(jacobian[0][r * 3 + i], jacobian1[r][i], 1e-4);
        }
    }

    for (int i = 0; i < 3; i++) {
        const double **compd;
        compd = new const double *[2];
        compd[0] = rot[0];
        compd[1] = trans_perturbed[i][0];

        thing.Evaluate(compd, residual1, jacobian);
        delete compd;

        for (int r = 0; r < 3; r++) {
            jacobian1[r][i] = (residual1[r] - residual[r]) / eps;
            EXPECT_NEAR(jacobian[1][r * 3 + i], jacobian1[r][i], 1e-4);
        }
    }
}

TEST(Residual_test, pointToPlaneAnalytic) {
    const double **trans;
    trans = new const double *;
    trans[0] = new const double[6]{0.5, 0.3, -0.2, 0.2, 5, 4};

    double **jacobian;
    jacobian = new double *;
    jacobian[0] = new double[6];

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, 0};
    double ptC[3] = {4, -1, 0};
    double pt[3] = {1, 2, -4};
    double scale = 1;
    double residual = 0;

    AnalyticalPointToPlane thing(pt, ptA, ptB, ptC, &scale);
    thing.Evaluate(trans, &residual, jacobian);

    EXPECT_NEAR(residual, 1.2087, 1e-4);
    EXPECT_NEAR(jacobian[0][0], 3.5577, 1e-4);
    EXPECT_NEAR(jacobian[0][1], -0.0643, 1e-4);
    EXPECT_NEAR(jacobian[0][2], 0.5962, 1e-4);
    EXPECT_NEAR(jacobian[0][3], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][4], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][5], 1, 1e-4);
}

// This test is for odometry for the car moving in a straight line through the garage
TEST(OdomTest, StraightLineGarage) {
    // Load entire sequence into memory
    std::vector<pcl::PointCloud<PointXYZIR>> clds;
    std::vector<pcl::PointCloud<PointXYZIR>::Ptr> cldptr;
    pcl::PCLPointCloud2 temp;
    pcl::PointCloud<PointXYZIR> temp2;
    LOG_INFO("Starting to load clouds");
    boost::filesystem::path p(TEST_SEQUENCE_DIR);
    std::vector<boost::filesystem::path> v;
    std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
    std::sort(v.begin(), v.end());
    int length = 0;
    for (auto iter = v.begin(); iter != v.end(); ++iter) {
        pcl::io::loadPCDFile(iter->string(), temp);
        pcl::fromPCLPointCloud2(temp, temp2);
        clds.push_back(temp2);
        pcl::PointCloud<PointXYZIR>::Ptr ptr(new pcl::PointCloud<PointXYZIR>);
        *ptr = clds.at(length);
        cldptr.push_back(ptr);
        length++;
    }

    LOG_INFO("Finished loading clouds");
    // odom setup
    LaserOdomParams params;
    params.n_flat = 20;
    params.n_edge = 20;
    params.max_correspondence_dist = 0.4;
    params.huber_delta = 0.2;
    params.opt_iters = 20;
    params.visualize = true;
    params.output_trajectory = true;
    //params.output_correspondences = true;
    params.rotation_stiffness = 1e-5;
    params.translation_stiffness = 5e-3;
    params.T_z_multiplier = 4;
    params.T_y_multiplier = 2;
    params.RP_multiplier = 20;
    params.imposePrior = true;
    LaserOdom odom(params);
    std::vector<PointXYZIR> vec;
    uint16_t prev_enc = 0;

    // Loop through pointclouds and send points grouped by encoder angle odom
    for (int i = 0; i < length; i++) {
        for (PointXYZIR pt : clds.at(i)) {
            PointXYZIR recovered;
            // unpackage intensity and encoder
            uint8_t *src = static_cast<uint8_t *>(static_cast<void *>(&(pt.intensity)));
            uint16_t encoder = *(static_cast<uint16_t *>(static_cast<void *>(src)));
            uint8_t intensity = *(static_cast<uint8_t *>(static_cast<void *>(src + 2)));

            // copy remaining fields
            recovered.x = pt.x;
            recovered.y = pt.y;
            recovered.z = pt.z;
            recovered.ring = pt.ring;
            recovered.intensity = intensity;
            if (encoder != prev_enc) {
                if (vec.size() > 0) {
                    std::chrono::microseconds dur(clds.at(i).header.stamp);
                    TimeType stamp(dur);
                    odom.addPoints(vec, prev_enc, stamp);
                    vec.clear();
                }
                prev_enc = encoder;
            }
            vec.emplace_back(recovered);
        }
    }
}

void dummyoutput(const TimeType * const stmap,
                 const std::array<double, 3> * const rot,
                 const std::array<double, 3> * const trans,
                 const pcl::PointCloud<pcl::PointXYZI> * const cld) {
    LOG_INFO("Got output!");
    LOG_INFO("%f", rot->at(0));
    LOG_INFO("%f", rot->at(1));
    LOG_INFO("%f", rot->at(2));
    LOG_INFO("%f", trans->at(0));
    LOG_INFO("%f", trans->at(1));
    LOG_INFO("%f", trans->at(2));
    LOG_INFO("Cloud size: %lu", cld->size());
}

// Output function test
TEST(OdomTest, OutputTest) {
    // Load entire sequence into memory
    std::vector<pcl::PointCloud<PointXYZIR>> clds;
    std::vector<pcl::PointCloud<PointXYZIR>::Ptr> cldptr;
    pcl::PCLPointCloud2 temp;
    pcl::PointCloud<PointXYZIR> temp2;
    for (int i = 0; i < 10; i++) {
        pcl::io::loadPCDFile(TEST_SEQUENCE_DIR + std::to_string(i) + ".pcd", temp);
        pcl::fromPCLPointCloud2(temp, temp2);
        clds.push_back(temp2);
        pcl::PointCloud<PointXYZIR>::Ptr ptr(new pcl::PointCloud<PointXYZIR>);
        *ptr = clds.at(i);
        cldptr.push_back(ptr);
    }

    // odom setup
    LaserOdomParams params;
    params.n_flat = 50;
    params.n_edge = 40;
    params.max_correspondence_dist = 0.4;
    params.huber_delta = 0.2;
    params.opt_iters = 5;
    params.rotation_stiffness = 1e-5;
    params.translation_stiffness = 5e-3;
    params.T_z_multiplier = 4;
    params.T_y_multiplier = 2;
    params.RP_multiplier = 20;
    params.imposePrior = true;
    LaserOdom odom(params);
    std::vector<PointXYZIR> vec;
    uint16_t prev_enc = 0;

    odom.registerOutputFunction(dummyoutput);

    // Loop through pointclouds and send points grouped by encoder angle odom
    for (int i = 0; i < 10; i++) {
        for (PointXYZIR pt : clds.at(i)) {
            PointXYZIR recovered;
            // unpackage intensity and encoder
            uint8_t *src = static_cast<uint8_t *>(static_cast<void *>(&(pt.intensity)));
            uint16_t encoder = *(static_cast<uint16_t *>(static_cast<void *>(src)));
            uint8_t intensity = *(static_cast<uint8_t *>(static_cast<void *>(src + 2)));

            // copy remaining fields
            recovered.x = pt.x;
            recovered.y = pt.y;
            recovered.z = pt.z;
            recovered.ring = pt.ring;
            recovered.intensity = intensity;
            if (encoder != prev_enc) {
                if (vec.size() > 0) {
                    std::chrono::microseconds dur(clds.at(i).header.stamp);
                    TimeType stamp(dur);
                    odom.addPoints(vec, prev_enc, stamp);
                    vec.clear();
                }
                prev_enc = encoder;
            }
            vec.emplace_back(recovered);
        }
    }
}

// This is less of a test and more something that can be
// played with to see how changing parameters affects which
// points are selected as keypoints

TEST_F(OdomTestFile, VisualizeFeatures) {
    PointCloudDisplay display("odom");
    display.startSpin();
    LaserOdomParams params;
    params.n_flat = 30;
    params.n_edge = 40;
    LaserOdom odom(params);
    std::vector<PointXYZIR> vec;
    pcl::PointCloud<pcl::PointXYZI>::Ptr vizref(new pcl::PointCloud<pcl::PointXYZI>),
      vizedge(new pcl::PointCloud<pcl::PointXYZI>), vizflats(new pcl::PointCloud<pcl::PointXYZI>);
    int counter = 1;
    auto timepoint = std::chrono::steady_clock::now();
    for (auto iter = this->ref.begin(); iter < this->ref.end(); iter++) {
        pcl::PointXYZI pt;
        pt.x = iter->x;
        pt.y = iter->y;
        pt.z = iter->z;
        pt.intensity = 0;
        vizref->push_back(pt);
        vec.push_back(*iter);
        if (counter % 12 == 0) {
            odom.addPoints(vec, 2000, timepoint);
            vec.clear();
        }
        counter++;
    }
    // Now add points with a tick of 0 to trigger feature extraction
    auto start = std::chrono::steady_clock::now();
    odom.addPoints(vec, 0, timepoint);
    auto extract_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
    LOG_INFO("Feature extraction took %lu ms", extract_time.count());

    for (uint16_t i = 0; i < 32; i++) {
        for (auto iter = odom.edges.at(i).begin(); iter < odom.edges.at(i).end(); iter++) {
            pcl::PointXYZI pt;
            pt.x = iter->pt[0];
            pt.y = iter->pt[1];
            pt.z = iter->pt[2];
            pt.intensity = 1;
            vizedge->push_back(pt);
        }
        for (auto iter = odom.flats.at(i).begin(); iter < odom.flats.at(i).end(); iter++) {
            pcl::PointXYZI pt;
            pt.x = iter->pt[0];
            pt.y = iter->pt[1];
            pt.z = iter->pt[2];
            pt.intensity = 1;
            pt.intensity = 2;
            vizedge->push_back(pt);
        }
    }

    display.addPointcloud(vizedge, 1);
    display.addPointcloud(vizflats, 2);
    cin.get();
    display.stopSpin();
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
