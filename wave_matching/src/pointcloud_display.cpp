#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/logging.hpp"

namespace wave {

PointcloudDisplay::PointcloudDisplay() {
    boost::mutex::scoped_lock cld_lock(this->add_cloud_mutex);
    boost::mutex::scoped_lock cld_update_lock(this->update_cloud_mutex);
    this->add_cloud = false;
    this->update_cloud = false;
    cld_lock.unlock();
    cld_update_lock.unlock();
}

void PointcloudDisplay::startSpin() {
    this->continueFlag.test_and_set(std::memory_order_relaxed);
    this->viewer_thread = new boost::thread(boost::bind(&PointcloudDisplay::spin, this));
}

void PointcloudDisplay::stopSpin() {
    this->continueFlag.clear(std::memory_order_relaxed);
    this->viewer_thread->join();
}

void PointcloudDisplay::spin() {
    // Initialize viewer
    this->viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer("Display"));
    this->viewer->initCameraParameters();
    this->viewer->setBackgroundColor(0, 0, 0);
    this->viewer->addCoordinateSystem(1.0);
    while(this->continueFlag.test_and_set(std::memory_order_relaxed) && !(this->viewer->wasStopped())) {
        this->viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));

        boost::mutex::scoped_lock cld_lock(this->add_cloud_mutex);
        if(this->add_cloud) {
            this->addCloudInternal();
        }
        cld_lock.unlock();
        boost::mutex::scoped_lock update_lock(this->add_cloud_mutex);
        if(this->update_cloud) {
            this->updateCloudInternal();
        }
        update_lock.unlock();
    }
    // Cleanup viewer
    this->viewer->close();
    this->viewer.reset();
}

void PointcloudDisplay::addPointcloud(const PCLPointCloud cld, int id) {
    boost::mutex::scoped_lock cld_lock(this->add_cloud_mutex);
    this->cloud_ = cld;
    this->id_ = id;
    this->add_cloud = true;
    cld_lock.unlock();
}

void PointcloudDisplay::updatePointcloud(const PCLPointCloud cld, int id) {
    boost::mutex::scoped_lock update_lock(this->add_cloud_mutex);
    this->cloud_ = cld;
    this->id_ = id;
    this->update_cloud = true;
    update_lock.unlock();
}

void PointcloudDisplay::addCloudInternal() {
    this->viewer->addPointCloud(this->cloud_, std::to_string(this->id_));
    this->add_cloud = false;
}

void PointcloudDisplay::updateCloudInternal() {
    this->viewer->updatePointCloud(this->cloud_, std::to_string(this->id_));
    this->update_cloud = false;
}

}
