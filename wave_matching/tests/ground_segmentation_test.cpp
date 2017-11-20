#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>

#include "wave/wave_test.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include "wave/matching/ground_segmentation.hpp"

namespace wave {

const auto TEST_SCAN = "tests/data/testscan.pcd";
const auto TEST_CONFIG = "tests/config/ground_segmentation.yaml";

TEST(ground_segmentation, how_to_use) {
    GroundSegmentationParams params(TEST_CONFIG);
    GroundSegmentation ground_segmentation(params);

    PCLPointCloudPtr ref, vizobs, vizground, vizdrv;

    ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    vizobs = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    vizground = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    vizdrv = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::PointCloud<PointXYZGD>::Ptr groundCloud =
      boost::make_shared<pcl::PointCloud<PointXYZGD>>();
    pcl::PointCloud<PointXYZGD>::Ptr obsCloud =
      boost::make_shared<pcl::PointCloud<PointXYZGD>>();
    pcl::PointCloud<PointXYZGD>::Ptr drvCloud =
      boost::make_shared<pcl::PointCloud<PointXYZGD>>();

    pcl::io::loadPCDFile(TEST_SCAN, *ref);

    // filter points on the car
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;

    pcl::ConditionOr<pcl::PointXYZ>::Ptr x_cond(
      new pcl::ConditionOr<pcl::PointXYZ>());
    pcl::ConditionOr<pcl::PointXYZ>::Ptr y_cond(
      new pcl::ConditionOr<pcl::PointXYZ>());
    pcl::ConditionOr<pcl::PointXYZ>::Ptr total_cond(
      new pcl::ConditionOr<pcl::PointXYZ>());

    x_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>(
        "x", pcl::ComparisonOps::LT, -3)));
    x_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 3)));

    y_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>(
        "y", pcl::ComparisonOps::LT, -1.1)));
    y_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>(
        "y", pcl::ComparisonOps::GT, 1.1)));

    total_cond->addCondition(x_cond);
    total_cond->addCondition(y_cond);

    condrem.setCondition(total_cond);

    condrem.setInputCloud(ref);
    condrem.filter(*ref);

    ground_segmentation.setupGroundSegmentation(
      ref, groundCloud, obsCloud, drvCloud);
    ground_segmentation.segmentGround();

    pcl::copyPointCloud(*groundCloud, *vizground);
    pcl::copyPointCloud(*obsCloud, *vizobs);
    pcl::copyPointCloud(*drvCloud, *vizdrv);

    PointCloudDisplay display("groundsegmenter");
    display.startSpin();
    display.addPointcloud(vizground, 0, true);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    display.addPointcloud(vizobs, 1);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    display.addPointcloud(vizdrv, 2);
    std::this_thread::sleep_for(std::chrono::seconds(10));

    display.stopSpin();
}

}  // namespace wave
