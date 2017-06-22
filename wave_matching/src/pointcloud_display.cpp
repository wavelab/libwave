#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/log.hpp"

namespace wave {

PointCloudDisplay::PointCloudDisplay(const std::string &name) {
    this->display_name = name;
}

void PointCloudDisplay::startSpin() {
    this->continueFlag.test_and_set(std::memory_order_relaxed);
    this->viewer_thread = std::thread(&PointCloudDisplay::spin, this);
}

void PointCloudDisplay::stopSpin() {
    this->continueFlag.clear(std::memory_order_relaxed);
    this->viewer_thread.join();
}

void PointCloudDisplay::spin() {
    // Initialize viewer
    this->viewer =
      std::make_shared<pcl::visualization::PCLVisualizer>(this->display_name);
    this->viewer->initCameraParameters();
    this->viewer->setBackgroundColor(0, 0, 0);
    this->viewer->addCoordinateSystem(1.0);
    while (this->continueFlag.test_and_set(std::memory_order_relaxed) &&
           !(this->viewer->wasStopped())) {
        this->viewer->spinOnce(3);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));

        this->update_mutex.lock();
        this->updateInternal();
        this->update_mutex.unlock();
    }
    // Cleanup viewer
    this->viewer->close();
    this->viewer.reset();
}

void PointCloudDisplay::addPointcloud(const PCLPointCloud &cld, int id) {
    this->update_mutex.lock();
    this->clouds.emplace(Cloud{cld, id});
    this->update_mutex.unlock();
}

void PointCloudDisplay::addPointcloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &cld, int id) {
    this->update_mutex.lock();
    this->cloudsi.emplace(CloudI{cld, id});
    this->update_mutex.unlock();
}

void PointCloudDisplay::addLine(const pcl::PointXYZ &pt1,
                                const pcl::PointXYZ &pt2,
                                int id1,
                                int id2) {
    this->update_mutex.lock();
    this->lines.emplace(Line{pt1, pt2, id1, id2});
    this->update_mutex.unlock();
}

void PointCloudDisplay::updateInternal() {
    // add or update clouds in the viewer until the queue is empty
    while (this->clouds.size() != 0) {
        const auto &cld = this->clouds.front();
        if (this->viewer->contains(std::to_string(cld.id))) {
            this->viewer->updatePointCloud(cld.cloud, std::to_string(cld.id));
        } else {
            this->viewer->addPointCloud(cld.cloud, std::to_string(cld.id));
        }
        this->clouds.pop();
    }

    while (this->cloudsi.size() != 0) {
        const auto &cld = this->cloudsi.front();
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
          col_handler(cld.cloud, "intensity");
        if (this->viewer->contains(std::to_string(cld.id))) {
            this->viewer->updatePointCloud(
              cld.cloud, col_handler, std::to_string(cld.id));
        } else {
            this->viewer->addPointCloud(
              cld.cloud, col_handler, std::to_string(cld.id));
        }
        this->cloudsi.pop();
    }

    double rad = 0.2;
    double hi = 200;
    double low = 0;

    while (this->lines.size() != 0) {
        const auto &line = this->lines.front();
        if (this->viewer->contains(std::to_string(line.id1) + "pt")) {
            this->viewer->updateSphere(
              line.pt1, rad, hi, low, low, std::to_string(line.id1) + "pt");
        } else {
            this->viewer->addSphere(
              line.pt1, rad, hi, low, low, std::to_string(line.id1) + "pt");
        }
        if (this->viewer->contains(std::to_string(line.id2) + "pt")) {
            this->viewer->updateSphere(
              line.pt2, rad, hi, low, low, std::to_string(line.id2) + "pt");
        } else {
            this->viewer->addSphere(
              line.pt2, rad, hi, low, low, std::to_string(line.id2) + "pt");
        }
        // Doesn't seem to be a way to update lines
        this->viewer->addLine(
          line.pt1,
          line.pt2,
          low,
          hi,
          low,
          std::to_string(line.id1) + std::to_string(line.id2) + "ln");
        this->lines.pop();
    }
}
}
