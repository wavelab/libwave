#ifndef WAVE_POINTCLOUDDISPLAY_HPP
#define WAVE_POINTCLOUDDISPLAY_HPP

#include <atomic>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "wave/matching/pcl_common.hpp"

namespace wave {

class PointcloudDisplay {
 public:
    PointcloudDisplay();
    void startSpin();
    void stopSpin();
    void addPointcloud(PCLPointCloud cld);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

 private:
    void spin();
    boost::thread *thread;
    std::atomic_flag continueFlag;
};

}  // namespace wave

#endif //WAVE_POINTCLOUDDISPLAY_HPP
