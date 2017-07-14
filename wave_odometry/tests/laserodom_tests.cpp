#include <pcl/io/pcd_io.h>
#include <chrono>
#include <pcap.h>
#include <string>
#include "wave/wave_test.hpp"
#include "wave/utils/log.hpp"
#include "wave/odometry/LaserOdom.hpp"
#include "wave/odometry/laser_odom_residuals.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace wave {

const std::string TEST_SCAN = "data/testscan.pcd";
const std::string TEST_SEQUENCE_DIR = "data/garage/";
const int sequence_length = 20;

// Fixture to load same pointcloud all the time
class OdomTestFile : public testing::Test {
 protected:
    OdomTestFile() {}

    virtual ~OdomTestFile() {}

    virtual void SetUp() {
        pcl::io::loadPCDFile(TEST_SCAN, (this->ref));
        // filter out points on the car
        for(auto iter = this->ref.begin(); iter < this->ref.end(); iter++) {
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
    for (int i = 0; i<sequence_length; i++) {
        pcl::io::loadPCDFile(TEST_SEQUENCE_DIR + std::to_string(i) + ".pcd", temp);
        clds.push_back(temp);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZI>);

        for(size_t j = 0; j < clds.at(i).size(); j++) {
            float packed = clds.at(i).at(j).intensity;
            uint16_t encoder = *static_cast<uint16_t*>(static_cast<void*>(&packed));
            clds.at(i).at(j).intensity = static_cast<float>(encoder);
        }
        *ptr = clds.at(i);
        cldptr.push_back(ptr);
    }

    for(int i = 0; i<sequence_length; i++) {
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
    uint8_t *dest = static_cast<uint8_t*>(static_cast<void*>(&packed));
    memcpy(dest, &angle, 2);
    memcpy(dest + 2, &intensity, 1);
    uint16_t recovered_angle = *static_cast<uint16_t*>(static_cast<void*>(dest));
    uint8_t recovered_in = *static_cast<uint8_t*>(static_cast<void*>(dest + 2));
    ASSERT_EQ(angle, recovered_angle);
    ASSERT_EQ(intensity, recovered_in);
}

// Check that residuals are providing correct distance with no transform parameters
TEST(Residual_test, pointToLine) {
    double trans[6] = {0, 0, 0, 0, 0, 0};
    double ptA[3] = {2, 1, 4};
    double ptB[3] = {-3, 1, 3.4};
    double pt[3] = {1, 2, -4};
    double scale = 0.5;
    double residual = 0;

    PointToLineError error;
    error(trans, pt, ptA, ptB, &scale, &residual);

    // Correct value coming from matlab calculation
    ASSERT_NEAR(residual, 7.8875, 0.0001);
}

TEST(Residual_test, pointToPlane) {
    double trans[6] = {0, 0, 0, 0, 0, 0};
    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, 0};
    double ptC[3] = {4, -1, 0};
    double pt[3] = {1, 2, -4};
    double scale = 0.5;
    double residual = 0;

    PointToPlaneError error;
    error(trans, pt, ptA, ptB, ptC, &scale, &residual);

    // Correct value coming from matlab calculation
    ASSERT_NEAR(std::fabs(residual), 4, 0.0001);
}

TEST(Residual_test, pointToLineAnalytic) {
    const double **trans;
    trans = new const double*;
    trans[0] = new const double[6]{0.5, 0.3, -0.2, 0.2, 5, 4};

    double **jacobian;
    jacobian = new double*;
    jacobian[0] = new double[6];

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, 0};
    double pt[3] = {1, 2, -4};
    double scale = 1;
    double residual = 0;

    AnalyticalPointToLine thing(pt, ptA, ptB, &scale);
    thing.Evaluate(trans, &residual, jacobian);

    EXPECT_NEAR(residual, 1.2394, 1e-4);
    EXPECT_NEAR(jacobian[0][0], 3.2963, 1e-4);
    EXPECT_NEAR(jacobian[0][1], 0.7104, 1e-4);
    EXPECT_NEAR(jacobian[0][2], 1.1772, 1e-4);
    EXPECT_NEAR(jacobian[0][3], -0.2212, 1e-4);
    EXPECT_NEAR(jacobian[0][4], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][5], 0.9752, 1e-4);

}

// This test is for odometry in approximately stationary scans
TEST(OdomTest, StationaryLab) {
    // Load entire sequence into memory
    std::vector<pcl::PointCloud<PointXYZIR>> clds;
    std::vector<pcl::PointCloud<PointXYZIR>::Ptr> cldptr;
    pcl::PCLPointCloud2 temp;
    pcl::PointCloud<PointXYZIR> temp2;
    for (int i = 0; i<sequence_length; i++) {
        pcl::io::loadPCDFile(TEST_SEQUENCE_DIR + std::to_string(i) + ".pcd", temp);
        pcl::fromPCLPointCloud2(temp, temp2);
        clds.push_back(temp2);
        pcl::PointCloud<PointXYZIR>::Ptr ptr(new pcl::PointCloud<PointXYZIR>);
        *ptr = clds.at(i);
        cldptr.push_back(ptr);
    }

    // odom setup
    LaserOdomParams params;
    params.n_flat = 20;
    params.n_edge = 10;
    params.max_correspondence_dist = 5;
    params.visualize = true;
    LaserOdom odom(params);
    std::vector<PointXYZIR> vec;
    uint16_t prev_enc = 0;

    // Loop through pointclouds and send points grouped by encoder angle odom
    bool initialized = false;
    double prev_transform[6];
    double cur_transform[6];
    for (int i = 0; i<sequence_length; i++) {
        for (PointXYZIR pt : clds.at(i)) {
            PointXYZIR recovered;
            // unpackage intensity and encoder
            uint8_t* src = static_cast<uint8_t*>(static_cast<void*>(&(pt.intensity)));
            uint16_t encoder = *(static_cast<uint16_t*>(static_cast<void*>(src)));
            uint8_t intensity = *(static_cast<uint8_t*>(static_cast<void*>(src + 2)));

            // copy remaining fields
            recovered.x = pt.x;
            recovered.y = pt.y;
            recovered.z = pt.z;
            recovered.ring = pt.ring;
            recovered.intensity = intensity;
            if (encoder != prev_enc) {
                if(vec.size() > 0) {
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
    params.n_flat = 20;
    params.n_edge = 20;
    LaserOdom odom(params);
    std::vector<PointXYZIR> vec;
    pcl::PointCloud<pcl::PointXYZI>::Ptr vizref(
      new pcl::PointCloud<pcl::PointXYZI>),
      vizedge(new pcl::PointCloud<pcl::PointXYZI>),
      vizflats(new pcl::PointCloud<pcl::PointXYZI>);
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
            odom.addPoints(vec, 1, timepoint);
            vec.clear();
        }
        counter++;
    }
    // Now add points with a tick of 0 to trigger feature extraction
    auto start = std::chrono::steady_clock::now();
    odom.addPoints(vec, 0, timepoint);
    auto extract_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start);
    LOG_INFO("Feature extraction took %lu ms", extract_time.count());

    for (auto iter = odom.edges.begin(); iter < odom.edges.end(); iter++) {
        pcl::PointXYZI pt;
        pt.x = iter->pt[0];
        pt.y = iter->pt[1];
        pt.z = iter->pt[2];
        pt.intensity = 1;
        vizedge->push_back(pt);
    }
    for (auto iter = odom.flats.begin(); iter < odom.flats.end(); iter++) {
        pcl::PointXYZI pt;
        pt.x = iter->pt[0];
        pt.y = iter->pt[1];
        pt.z = iter->pt[2];
        pt.intensity = 1;
        pt.intensity = 2;
        vizedge->push_back(pt);
    }

    display.addPointcloud(vizedge, 1);
    display.addPointcloud(vizflats, 2);
    cin.get();
    display.stopSpin();
}

}  // namespace wave
