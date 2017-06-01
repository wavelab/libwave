#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "wave/wave_test.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/logging.hpp"
#include "wave/utils/math.hpp"

namespace wave {

const auto TEST_SCAN = "data/testscan.pcd";

TEST(viewer, init_test) {
    PointCloudDisplay display("init_test");
    display.startSpin();
    boost::this_thread::sleep(
      boost::posix_time::microseconds(5000000));  // five seconds
    display.stopSpin();
}

TEST(viewer, pointcloud_test) {
    PointCloudDisplay display("pointcloud_test");
    display.startSpin();
    PCLPointCloud cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile(TEST_SCAN, *cloud);
    display.addPointcloud(cloud, 1);
    boost::this_thread::sleep(
      boost::posix_time::microseconds(5000000));  // five seconds
    display.stopSpin();
}

TEST(viewer, multiple_clouds_test) {
    PointCloudDisplay display("multiple_clouds_test");
    display.startSpin();
    PCLPointCloud cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile(TEST_SCAN, *cloud);
    display.addPointcloud(cloud, 1);
    Eigen::Affine3f transform(Eigen::Translation3f(Eigen::Vector3f(20, 0, 0)));
    for (int i = 0; i < 3; i++) {
        pcl::transformPointCloud(*cloud, *cloud, transform);
        display.addPointcloud(cloud, i + 2);
        boost::this_thread::sleep(
          boost::posix_time::microseconds(1000000));  // one seconds
    }
    boost::this_thread::sleep(
      boost::posix_time::microseconds(5000000));  // five seconds
    display.stopSpin();
}

TEST(viewer, line_test) {
    PointCloudDisplay display("line_test");
    display.startSpin();
    std::vector<pcl::PointXYZ> pts(10);
    for (int i = 0; i < 10; i++) {
        pts.at(i) = pcl::PointXYZ(cos(i), sin(i), 0.3 * i);
    }
    for (int j = 0; j < 9; j++) {
        display.addLine(pts.at(j), pts.at(j + 1), j, j + 1);
    }
    boost::this_thread::sleep(
      boost::posix_time::microseconds(5000000));  // five seconds
    display.stopSpin();
}

}  // namespace wave
