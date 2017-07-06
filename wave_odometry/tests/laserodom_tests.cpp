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

TEST(laserodom, VizSequence) {
    // Playback in visualizer
    PointCloudDisplay display("sequence");
    display.startSpin();
    // Put entire sequence in memory
    std::vector<pcl::PointCloud<pcl::PointXYZI>> clds;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cldptr;
    pcl::PointCloud<pcl::PointXYZI> temp;
    for (int i = 0; i<37; i++) {
        pcl::io::loadPCDFile(TEST_SEQUENCE_DIR + std::to_string(i) + ".pcd", temp);
        clds.push_back(temp);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZI>);
        *ptr = clds.at(i);
        cldptr.push_back(ptr);
    }

    for(int i = 0; i<37; i++) {
        display.addPointcloud(cldptr.at(i), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    display.stopSpin();
}

//TEST(OdomTest, StationaryLab) {
//    // PCAP setup
//    char errbuff[PCAP_ERRBUF_SIZE];
//    pcap_t * pcap = pcap_open_offline(PCD_DIR.c_str(), errbuff);
//    struct pcap_pkthdr *header;
//    const u_char *data;
//    u_int packetCount = 0;
//
//    // odom setup
//    LaserOdomParams params;
//    LaserOdom odom(params);
//    std::vector<PointXYZIR> vec;
//
//    // Loop through and pass any point packets through to odom
//    while (int returnValue = pcap_next_ex(pcap, &header, &data) >= 0) {
//        if (header->len == 1248ul) {
//            // Have a velodyne packet!
//
//        }
//    }
//}

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
