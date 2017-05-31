#ifndef WAVE_POINTCLOUDDISPLAY_HPP
#define WAVE_POINTCLOUDDISPLAY_HPP

#include <atomic>
#include <queue>
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
    void addLine(pcl::PointXYZ pt1, pcl::PointXYZ pt2, int id1, int id2);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

 private:
    void updateInternal();
    void spin();
    boost::thread *viewer_thread;
    std::atomic_flag continueFlag = ATOMIC_FLAG_INIT;
    boost::mutex update_mutex;
    struct Cloud {
        PCLPointCloud cloud;
        int id;
    };
    struct Line {
        pcl::PointXYZ pt1, pt2;
        int id1, id2;
    };
    std::queue<Cloud> clouds;
    std::queue<Line> lines;
};

}  // namespace wave

#endif  // WAVE_POINTCLOUDDISPLAY_HPP
