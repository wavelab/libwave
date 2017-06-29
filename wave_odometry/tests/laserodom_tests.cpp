#include <pcl/io/pcd_io.h>
#include "wave/wave_test.hpp"
#include "wave/odometry/LaserOdom.hpp"

namespace wave {

const auto TEST_SCAN = "data/testscan.pcd";

// Fixture to load same pointcloud all the time
class OdomTestFile : public testing::Test {
 protected:
    OdomTestFile() {}

    virtual ~OdomTestFile() {}

    virtual void SetUp() {
        pcl::io::loadPCDFile(TEST_SCAN, (this->ref));
    }
    pcl::PointCloud<PointXYZIR> ref;
};

TEST(laserodom, Init) {
    LaserOdom odom(LaserOdomParams());
}

TEST_F(OdomTestFile, LoadPCD) {
    LaserOdomParams params;
    LaserOdom odom(params);
    std::vector<PointXYZIR> vec;
    int counter = 0;
    auto timepoint = std::chrono::steady_clock::now();
    for (auto iter = this->ref.begin(); iter < this->ref.end(); iter++) {
        PointXYZIR pt;
        pt.x = iter->x;
        pt.y = iter->y;
        pt.z = iter->z;
        pt.intensity = iter->intensity;
        pt.ring = iter->ring;
        vec.push_back(pt);
        if (counter == 0) {
            odom.addPoints(vec, 0, timepoint);
            vec.clear();
        } else if (counter%12 == 0) {
            odom.addPoints(vec, 1, timepoint);
            vec.clear();
        }
        counter++;
    }
    // Now add points with a tick of 0 to trigger feature extraction
    odom.addPoints(vec, 0, timepoint);
    pcl::io::savePCDFile("edges", odom.edges);
    pcl::io::savePCDFile("flats", odom.flats);
}

}  // namespace wave
