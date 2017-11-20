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
    this->viewer->setBackgroundColor(0, 0, 0);
    this->viewer->addCoordinateSystem(1.0);
    this->viewer->resetCamera();
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

void PointCloudDisplay::addPointcloud(const PCLPointCloud &cld,
                                      int id,
                                      bool reset_camera) {
    this->update_mutex.lock();
    this->clouds.emplace(Cloud{cld->makeShared(), id, reset_camera});
    this->update_mutex.unlock();
}

void PointCloudDisplay::addPointcloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &cld, int id, bool reset_camera) {
    this->update_mutex.lock();
    this->cloudsi.emplace(CloudI{cld->makeShared(), id, reset_camera});
    this->update_mutex.unlock();
}

void PointCloudDisplay::addLine(const pcl::PointXYZ &pt1,
                                const pcl::PointXYZ &pt2,
                                int id1,
                                int id2,
                                bool reset_camera) {
    this->update_mutex.lock();
    this->lines.emplace(Line{pt1, pt2, id1, id2, reset_camera});
    this->update_mutex.unlock();
}

void PointCloudDisplay::updateInternal() {
    // add or update clouds in the viewer until the queue is empty
    while (this->clouds.size() != 0) {
        const auto &cld = this->clouds.front();
        // Give each id a unique color, using the Glasbey table of maximally
        // different colors. Use white for 0
        // Note the color handler uses the odd format of doubles 0-255
        Eigen::Vector3d rgb{255., 255., 255.};
        if (cld.id > 0 &&
            static_cast<unsigned>(cld.id) < pcl::GlasbeyLUT::size()) {
            rgb = pcl::GlasbeyLUT::at(cld.id).getRGBVector3i().cast<double>();
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          col_handler{cld.cloud, rgb[0], rgb[1], rgb[2]};

        if (this->viewer->contains(std::to_string(cld.id))) {
            this->viewer->updatePointCloud(
              cld.cloud, col_handler, std::to_string(cld.id));
        } else {
            this->viewer->addPointCloud(
              cld.cloud, col_handler, std::to_string(cld.id));
        }
        if (cld.reset_camera) {
            this->viewer->resetCamera();
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
        if (cld.reset_camera) {
            this->viewer->resetCamera();
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
        if (line.reset_camera) {
            this->viewer->resetCamera();
        }
        this->lines.pop();
    }
}
}
