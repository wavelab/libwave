#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/logging.hpp"

namespace wave {

PointcloudDisplay::PointcloudDisplay() {
    boost::mutex::scoped_lock cld_update_lock(this->update_cloud_mutex);
    this->update_cloud = false;
    cld_update_lock.unlock();
}

void PointcloudDisplay::startSpin() {
    this->continueFlag.test_and_set(std::memory_order_relaxed);
    this->viewer_thread =
      new boost::thread(boost::bind(&PointcloudDisplay::spin, this));
}

void PointcloudDisplay::stopSpin() {
    this->continueFlag.clear(std::memory_order_relaxed);
    this->viewer_thread->join();
}

void PointcloudDisplay::spin() {
    // Initialize viewer
    this->viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
      new pcl::visualization::PCLVisualizer("Display"));
    this->viewer->initCameraParameters();
    this->viewer->setBackgroundColor(0, 0, 0);
    this->viewer->addCoordinateSystem(1.0);
    while (this->continueFlag.test_and_set(std::memory_order_relaxed) &&
           !(this->viewer->wasStopped())) {
        this->viewer->spinOnce(1);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));

        boost::mutex::scoped_lock cld_lock(this->update_cloud_mutex);
        if (this->update_cloud) {
            this->addCloudInternal();
        }
        cld_lock.unlock();
    }
    // Cleanup viewer
    this->viewer->close();
    this->viewer.reset();
}

void PointcloudDisplay::addPointcloud(const PCLPointCloud cld, int id) {
    boost::mutex::scoped_lock cld_lock(this->update_cloud_mutex);
    this->clouds.emplace(Cloud{cld, id});
    this->update_cloud = true;
    cld_lock.unlock();
}

void PointcloudDisplay::addCloudInternal() {
    // add or update clouds in the viewer until the queue is empty
    while (this->clouds.size() != 0) {
        if (this->viewer->contains(std::to_string(this->clouds.front().id))) {
            this->viewer->updatePointCloud(
              this->clouds.front().cloud,
              std::to_string(this->clouds.front().id));
        } else {
            this->viewer->addPointCloud(
              this->clouds.front().cloud,
              std::to_string(this->clouds.front().id));
        }
        this->clouds.pop();
    }
    this->update_cloud = false;
}
}
