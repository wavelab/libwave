#include <pcl/io/pcd_io.h>

#include "wave/wave_test.hpp"
#include "wave/matching/pcl_common.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include "wave/matching/groundSegmentation.hpp"

namespace wave {

const auto TEST_SCAN = "data/testscan.pcd";

// Fixture to load same pointcloud all the time
class GSTest : public testing::Test {
 protected:
    GSTest() {}

    virtual ~GSTest() {}

    virtual void SetUp() {
        this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPCDFile(TEST_SCAN, *(this->ref));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ref;
};

TEST(GroundSeg, initialization) {
    GroundSegmentationParams params;
    groundSegmentation segment(params);
}

TEST_F(GSTest, segment) {
    GroundSegmentationParams params;
    groundSegmentation segment(params);
    pcl::PointCloud<PointXYZGD>::Ptr groundCloud =
            boost::make_shared<pcl::PointCloud<PointXYZGD>>();
    pcl::PointCloud<PointXYZGD>::Ptr obsCloud =
            boost::make_shared<pcl::PointCloud<PointXYZGD>>();
    pcl::PointCloud<PointXYZGD>::Ptr drvCloud =
            boost::make_shared<pcl::PointCloud<PointXYZGD>>();

    segment.setupGroundSegmentation(this->ref, groundCloud, obsCloud, drvCloud);
    segment.segmentGround();

    PCLPointCloud output = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::copyPointCloud (*obsCloud, *output);

    PointCloudDisplay display("segmented");
    display.startSpin();
    display.addPointcloud(this->ref, 0);

    while(cin.get() != 'c') ;

    display.addPointcloud(output, 0);

    while(cin.get() != 'c');
    display.stopSpin();
}

}  // namespace wave
