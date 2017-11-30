#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>

#include "wave/wave_test.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include "wave/matching/ground_segmentation.hpp"

namespace wave {

const auto TEST_SCAN = "tests/data/testscan.pcd";
const auto TEST_CONFIG = "tests/config/ground_segmentation.yaml";

TEST(ground_segmentation, how_to_use) {
    // Prepare the input
    // @todo factor out pre-filtering steps

    PCLPointCloudPtr input =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto vizground = input->makeShared();
    auto vizobs = input->makeShared();
    auto vizdrv = input->makeShared();


    pcl::io::loadPCDFile(TEST_SCAN, *input);

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

    condrem.setInputCloud(input);
    condrem.filter(*input);

    // Test the actual ground segmentation filter
    GroundSegmentationParams params{TEST_CONFIG};
    GroundSegmentation<pcl::PointXYZ> ground_segmentation{params};
    ground_segmentation.setInputCloud(input);

    ground_segmentation.setKeepGround(true);
    ground_segmentation.setKeepObstacle(false);
    ground_segmentation.setKeepOverhanging(false);
    ground_segmentation.filter(*vizground);

    ground_segmentation.setKeepGround(false);
    ground_segmentation.setKeepObstacle(true);
    ground_segmentation.filter(*vizobs);

    ground_segmentation.setKeepObstacle(false);
    ground_segmentation.setKeepOverhanging(true);
    ground_segmentation.filter(*vizdrv);

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
