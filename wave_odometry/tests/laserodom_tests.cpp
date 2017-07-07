#include <pcl/io/pcd_io.h>
#include <chrono>
#include <pcap.h>
#include <string>
#include "wave/wave_test.hpp"
#include "wave/utils/log.hpp"
#include "wave/odometry/LaserOdom.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace wave {

const std::string TEST_SCAN = "data/testscan.pcd";
const std::string PCD_DIR = "data/lab.pcap";
const std::string TEST_SEQUENCE_DIR = "data/sequence/";
const int sequence_length = 35;

union PackagedInfo {
    float intensity;  // 4 bytes
    uint16_t s[2];    // 4 bytes
};

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
            PackagedInfo info;
            info.intensity = clds.at(i).at(j).intensity;

            LOG_INFO("Encoder is %i", info.s[1]);
            clds.at(i).at(j).intensity = static_cast<float>(info.s[1]);
        }
        *ptr = clds.at(i);
        cldptr.push_back(ptr);
    }

    for(int i = 0; i<sequence_length; i++) {
        display.addPointcloud(cldptr.at(i), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    display.stopSpin();
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
    LaserOdom odom(params);
    std::vector<PointXYZIR> vec;
    uint16_t prev_enc = 0;

    // Loop through pointclouds and send points grouped by encoder angle odom
    for (int i = 0; i<sequence_length; i++) {
        for (PointXYZIR pt : clds.at(i)) {
            PointXYZIR recovered;
            // unpackage intensity and encoder
            PackagedInfo info;
            info.intensity = pt.intensity;
            printf("Encoder is %i", info.s[1]);

            // copy remaining fields
            memcpy(recovered.data, pt.data, 4);
            recovered.ring = pt.ring;
            recovered.intensity = static_cast<float>(info.s[0]);
            printf("\n");
            if (info.s[1] != prev_enc) {
                if(vec.size() > 0) {
                    std::chrono::microseconds dur(clds.at(i).header.stamp);
                    TimeType stamp(dur);
                    odom.addPoints(vec, prev_enc, stamp);
                    vec.clear();
                }
                prev_enc = info.s[1];
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
