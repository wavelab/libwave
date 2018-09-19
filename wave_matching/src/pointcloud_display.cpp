#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/log.hpp"

namespace wave {

PointCloudDisplay::PointCloudDisplay(const std::string &name, double rad, int viewport_cnt_x, int viewport_cnt_y) :
    viewport_cnt_x(viewport_cnt_x), viewport_cnt_y(viewport_cnt_y) {
    this->display_name = name;
    this->radius = rad;
}

void PointCloudDisplay::removeAll() {
    this->reset_shapes.clear(std::memory_order_relaxed);
    this->reset_clouds.clear(std::memory_order_relaxed);
}

void PointCloudDisplay::removeAllShapes() {
    this->reset_shapes.clear(std::memory_order_relaxed);
}

void PointCloudDisplay::removeAllClouds() {
    this->reset_clouds.clear(std::memory_order_relaxed);
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
    double x_increment = 1.0 / (double) this->viewport_cnt_x;
    double y_increment = 1.0 / (double) this->viewport_cnt_y;

    int viewport_id = 1;
    for (int x_idx = 0; x_idx < viewport_cnt_x; ++x_idx) {
        for (int y_idx = 0; y_idx < viewport_cnt_y; ++y_idx) {
            double x_min = x_idx * x_increment;
            double y_min = y_idx * y_increment;
            double x_max = x_min + x_increment;
            double y_max = y_min + y_increment;
            if (x_idx + 1 == viewport_cnt_x) {
                x_max = 1.0;
            }
            if (y_idx + 1 == viewport_cnt_y) {
                y_max = 1.0;
            }
            this->viewer->createViewPort(x_min, y_min, x_max, y_max, viewport_id);
            this->viewer->addText("Viewport " + std::to_string(viewport_id), 10, 10, "Vp text " + std::to_string(viewport_id), viewport_id);
            std::string unique_id = "Viewport " + std::to_string(viewport_id);
            this->viewer->addCoordinateSystem(1.0, unique_id, viewport_id);
            ++viewport_id;
        }
    }
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
                                      bool reset_camera,
                                      int viewport) {
    this->update_mutex.lock();
    this->clouds.emplace(Cloud{cld->makeShared(), id, viewport, reset_camera});
    this->update_mutex.unlock();
}

void PointCloudDisplay::addPointcloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &cld, int id, bool reset_camera, int viewport) {
    this->update_mutex.lock();
    this->cloudsi.emplace(CloudI{cld->makeShared(), id, viewport, reset_camera});
    this->update_mutex.unlock();
}

void PointCloudDisplay::addLine(const pcl::PointXYZ &pt1,
                                const pcl::PointXYZ &pt2,
                                int id1,
                                int id2,
                                bool reset_camera,
                                int viewport,
                                uint8_t colour) {
    this->update_mutex.lock();
    this->lines.emplace(Line{pt1, pt2, id1, id2, viewport, colour, reset_camera});
    this->update_mutex.unlock();
}

void PointCloudDisplay::addSquare(const pcl::PointXYZ &pt0, const pcl::PointXYZ &dir, float l1dist, int id,
                                  bool reset_camera, int viewport, uint8_t colour) {
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
    
    contour.emplace_back(pcl::PointXYZ(- offset, - offset, 0));
    contour.emplace_back(pcl::PointXYZ(- offset, + offset, 0));
    contour.emplace_back(pcl::PointXYZ(offset, offset, 0));
    contour.emplace_back(pcl::PointXYZ(offset, - offset, 0));

    Eigen::Map<const Eigen::Vector3f> point(pt0.data);
    for (auto &elem : contour) {
        Eigen::Map<Eigen::Vector3f> map(elem.data);
        map = point + R * map;
    }

    Eigen::Vector4f coefficients;
    coefficients(0) = dir.x;
    coefficients(1) = dir.y;
    coefficients(2) = dir.z;
    coefficients(3) = -pt0.x * dir.x - pt0.y * dir.y - pt0.z * dir.z;

    pcl::PlanarPolygon<pcl::PointXYZ> square_poly(contour, coefficients);

    this->update_mutex.lock();
    this->squares.emplace(Square{square_poly, id, viewport, colour, reset_camera});
    this->update_mutex.unlock();
}

void PointCloudDisplay::updateInternal() {
    if(!(this->reset_shapes.test_and_set(std::memory_order_relaxed))) {
        this->viewer->removeAllShapes();
        int viewport_id = 1;
        for (int x_idx = 0; x_idx < viewport_cnt_x; ++x_idx) {
            for (int y_idx = 0; y_idx < viewport_cnt_y; ++y_idx) {
                this->viewer->addText("Viewport " + std::to_string(viewport_id), 10, 10, "Vp text " + std::to_string(viewport_id), viewport_id);
                ++viewport_id;
            }
        }
    }
    if (!(this->reset_clouds.test_and_set(std::memory_order_relaxed))) {
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
            if (cld.viewport > this->viewport_cnt_x * this->viewport_cnt_y)
                throw std::runtime_error("Viewport index too big");
            this->viewer->addPointCloud(
              cld.cloud, col_handler, std::to_string(cld.id), cld.viewport);
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
            if (cld.viewport > this->viewport_cnt_x * this->viewport_cnt_y)
                throw std::runtime_error("Viewport index too big");
            this->viewer->addPointCloud(
              cld.cloud, col_handler, std::to_string(cld.id), cld.viewport);
        }
        if (cld.reset_camera) {
            this->viewer->resetCamera();
        }
        this->cloudsi.pop();
    }

    while (!this->lines.empty()) {
        const auto &line = this->lines.front();
        // Doesn't seem to be a way to update lines
        auto lineid =
          std::to_string(line.id1) + std::to_string(line.id2) + "ln";
        if (this->viewer->contains(lineid)) {
            this->viewer->removeShape(lineid, 0);
        }
        if (line.viewport > this->viewport_cnt_x * this->viewport_cnt_y)
            throw std::runtime_error("Viewport index too big");
        double red = (double)(line.colour) / 255.0;
        double blue = 1.0 - red;
        this->viewer->addLine(line.pt1, line.pt2, red, 0, blue, lineid, line.viewport);
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
        if (square.viewport > this->viewport_cnt_x * this->viewport_cnt_y)
            throw std::runtime_error("Viewport index too big");
        double red = (double)(square.colour) / 255.0;
        double blue = 1.0 - red;
        this->viewer->addPolygon(square.planar_poly, red, 0.0, blue, squareid, square.viewport);
        if (square.reset_camera) {
            this->viewer->resetCamera();
        }
        this->squares.pop();
    }
}
}
