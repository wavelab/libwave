#include "wave/matching/visualizer.hpp"

namespace wave {

PointcloudDisplay::PointcloudDisplay() {
    this->viewer = new pcl::visualization::PCLVisualizer();
    this->viewer->setBackgroundColor(0, 0, 0);
    this->viewer->initCameraParameters();
}

}
