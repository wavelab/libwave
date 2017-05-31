#include "wave/matching/pointcloud_display.hpp"

namespace wave {

PointcloudDisplay::PointcloudDisplay() {
    this->viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer());
    this->viewer->setBackgroundColor(0, 0, 0);
    this->viewer->addCoordinateSystem(1.0);
    this->viewer->initCameraParameters();
}

void PointcloudDisplay::startSpin() {
    this->continueFlag.test_and_set(std::memory_order_relaxed);
    this->thread = new boost::thread(boost::bind(&PointcloudDisplay::spin, this));
}

void PointcloudDisplay::stopSpin() {
    this->continueFlag.clear(std::memory_order_relaxed);
    this->thread->join();
}

void PointcloudDisplay::spin() {
    while(!(this->continueFlag.test_and_set(std::memory_order_relaxed)) && !(this->viewer->wasStopped())) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

}
