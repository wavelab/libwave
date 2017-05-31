#ifndef WAVE_VISUALIZER_HPP
#define WAVE_VISUALIZER_HPP

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

namespace wave {

class PointcloudDisplay {
 public:
    PointcloudDisplay();
    void start_spin();
    void stop_spin();
 private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


};

}  // namespace wave

#endif //WAVE_VISUALIZER_HPP
