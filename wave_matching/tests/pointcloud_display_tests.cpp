#include <pcl/io/pcd_io.h>

#include "wave/wave_test.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace wave {

const auto TEST_SCAN = "data/testscan.pcd";

TEST(viewer, init_test) {
    PointcloudDisplay display;
    display.startSpin();
    boost::this_thread::sleep(boost::posix_time::microseconds(1000000));  // one second
    display.stopSpin();
}

}  // namespace wave
