#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/logging.hpp"

namespace wave {

PointcloudDisplay::PointcloudDisplay() {}

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

        boost::mutex::scoped_lock cld_lock(this->update_mutex);
        this->updateInternal();
        cld_lock.unlock();
    }
    // Cleanup viewer
    this->viewer->close();
    this->viewer.reset();
}

void PointcloudDisplay::addPointcloud(const PCLPointCloud cld, int id) {
    boost::mutex::scoped_lock lock(this->update_mutex);
    this->clouds.emplace(Cloud{cld, id});
    lock.unlock();
}

void PointcloudDisplay::addLine(pcl::PointXYZ pt1,
                                pcl::PointXYZ pt2,
                                int id1,
                                int id2) {
    boost::mutex::scoped_lock lock(this->update_mutex);
    this->lines.emplace(Line{pt1, pt2, id1, id2});
    lock.unlock();
}

void PointcloudDisplay::updateInternal() {
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
    while (this->lines.size() != 0) {
        if (this->viewer->contains(std::to_string(this->lines.front().id1) +
                                   "pt")) {
            this->viewer->updateSphere(
              this->lines.front().pt1,
              0.2,
              200,
              0,
              0,
              std::to_string(this->lines.front().id1) + "pt");
        } else {
            this->viewer->addSphere(
              this->lines.front().pt1,
              0.2,
              200,
              0,
              0,
              std::to_string(this->lines.front().id1) + "pt");
        }
        if (this->viewer->contains(std::to_string(this->lines.front().id2) +
                                   "pt")) {
            this->viewer->updateSphere(
              this->lines.front().pt2,
              0.2,
              200,
              0,
              0,
              std::to_string(this->lines.front().id2) + "pt");
        } else {
            this->viewer->addSphere(
              this->lines.front().pt2,
              0.2,
              200,
              0,
              0,
              std::to_string(this->lines.front().id2) + "pt");
        }
        // Doesn't seem to be a way to update lines
        this->viewer->addLine(this->lines.front().pt1,
                              this->lines.front().pt2,
                              0,
                              200,
                              0,
                              std::to_string(this->lines.front().id1) +
                                std::to_string(this->lines.front().id2) + "ln");
        this->lines.pop();
    }
}
}
