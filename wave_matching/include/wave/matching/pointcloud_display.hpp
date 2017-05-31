#ifndef WAVE_POINTCLOUDDISPLAY_HPP
#define WAVE_POINTCLOUDDISPLAY_HPP

#include <atomic>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
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
    void addPointcloud(PCLPointCloud cld, int id);
    void updatePointcloud(PCLPointCloud cld, int id);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

 private:
    void addCloudInternal();
    void updateCloudInternal();
    void spin();
    boost::thread *viewer_thread;
    std::atomic_flag continueFlag = ATOMIC_FLAG_INIT;
    boost::mutex add_cloud_mutex, update_cloud_mutex;
    bool add_cloud, update_cloud;
    PCLPointCloud cloud_;
    int id_;
};

}  // namespace wave

#endif //WAVE_POINTCLOUDDISPLAY_HPP
