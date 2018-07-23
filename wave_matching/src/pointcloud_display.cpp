#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/log.hpp"

namespace wave {

PointCloudDisplay::PointCloudDisplay(const std::string &name, double rad) {
    this->display_name = name;
    this->radius = rad;
}

void PointCloudDisplay::removeAll() {
    this->resetLines.clear(std::memory_order_relaxed);
}

void PointCloudDisplay::startSpin() {
    this->continueFlag.test_and_set(std::memory_order_relaxed);
    this->viewer_thread = std::thread(&PointCloudDisplay::spin, this);
}

void PointCloudDisplay::stopSpin() {
    this->continueFlag.clear(std::memory_order_relaxed);
    this->viewer_thread.join();
}

PointCloudDisplay::~PointCloudDisplay() {
    this->continueFlag.clear(std::memory_order_relaxed);
    if(this->viewer_thread.joinable()) {
        this->viewer_thread.join();
    }
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

void PointCloudDisplay::addPointcloud(const PCLPointCloudPtr &cld,
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

void PointCloudDisplay::addSquare(const pcl::PointXYZ &pt0, const pcl::PointXYZ &dir, float l1dist, int id,
                                  bool reset_camera) {
    pcl::PointCloud<pcl::PointXYZ>::VectorType contour;
    float offset = 0.5f * l1dist;

    // Cross product of unit z vector with p
    Eigen::Vector2f axis;
    axis << -dir.y, dir.x; //, 0.0;

    double c = dir.z;

    Eigen::Matrix3f skew = Eigen::Matrix3f::Zero();
    skew(0, 2) = axis(1);
    skew(1, 2) = -axis(0);
    skew(2, 0) = -axis(1);
    skew(2, 1) = axis(0);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity() + skew + (1.0 / (1.0 + c)) * skew * skew;
    
    contour.emplace_back(pcl::PointXYZ(pt0.x - offset, pt0.y - offset, pt0.z));
    contour.emplace_back(pcl::PointXYZ(pt0.x - offset, pt0.y + offset, pt0.z));
    contour.emplace_back(pcl::PointXYZ(pt0.x + offset, pt0.y + offset, pt0.z));
    contour.emplace_back(pcl::PointXYZ(pt0.x + offset, pt0.y - offset, pt0.z));
    
    for (auto &elem : contour) {
        Eigen::Map<Eigen::Vector3f> map(elem.data);
        map = R * map;
    }

    Eigen::Vector4f coefficients;
    coefficients(0) = dir.x;
    coefficients(1) = dir.y;
    coefficients(2) = dir.z;
    coefficients(3) = -pt0.x * dir.x - pt0.y * dir.y - pt0.z * dir.z;

    pcl::PlanarPolygon<pcl::PointXYZ> square_poly(contour, coefficients);

    this->update_mutex.lock();
    this->squares.emplace(Square{square_poly, id, reset_camera});
    this->update_mutex.unlock();
}

void PointCloudDisplay::updateInternal() {
    if(!(this->resetLines.test_and_set(std::memory_order_relaxed))) {
        this->viewer->removeAllShapes();
        this->viewer->removeAllPointClouds();
    }
    // add or update clouds in the viewer until the queue is empty
    while (!this->clouds.empty()) {
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

    while (!this->cloudsi.empty()) {
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

    double hi = 200;
    double low = 0;

    while (!this->lines.empty()) {
        const auto &line = this->lines.front();
        // Doesn't seem to be a way to update lines
        auto lineid =
          std::to_string(line.id1) + std::to_string(line.id2) + "ln";
        if (this->viewer->contains(lineid)) {
            this->viewer->removeShape(lineid, 0);
        }
        this->viewer->addLine(line.pt1, line.pt2, low, hi, low, lineid);
        if (line.reset_camera) {
            this->viewer->resetCamera();
        }
        this->lines.pop();
    }

    while (!this->squares.empty()) {
        const auto &square = this->squares.front();

        auto squareid = std::to_string(square.id) + "sqr";

        if (this->viewer->contains(squareid)) {
            this->viewer->removeShape(squareid, 0);
        }
        //todo decide how to use rgb
        this->viewer->addPolygon(square.planar_poly, 1.0, 1.0, 1.0, squareid);
        if (square.reset_camera) {
            this->viewer->resetCamera();
        }
        this->squares.pop();
    }
}
}
