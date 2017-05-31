/** @file
 * @ingroup matching
 *
 * Interface to PCL Visualiser Class
 *
 * This is a class that provides an interface to PCLVisualiser class for
 * making cool graphics and debugging. The visualiser itself runs in a
 * separate thread so that it is always interactive.
 * Note: Ids are shared between all drawn objects, so the should be
 * globally unique.
 */

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
/** @addtogroup matching
 *  @{ */

class PointcloudDisplay {
 public:
    /**
     * Default constructor
     */
    PointcloudDisplay();

    /**
     * Calling this launches the worker thread and opens
     * the visualization window
     */
    void startSpin();

    /**
     * Calling this terminates the worker thread and
     * should close the window (note there is a bug in
     * vtk that causes the window to hang around)
     */
    void stopSpin();

    /**
     * Queues a pointcloud to be added to the display.
     * If worker thread isn't running nothing will happen.
     * @param cld: cloud to be added
     * @param id: id of cloud to be added. Same id can be
     * used to update cloud later
     */
    void addPointcloud(PCLPointCloud cld, int id);

    /**
     * Queues a line to be added to the display.
     * Note that three objects are added for one line:
     * spheres at the endpoints, and a line between them.
     * If worker thread isn't running nothing will happen.
     * @param pt1 Start coordinate of line
     * @param pt2 End coordinate of line
     * @param id1 Start point ID
     * @param id2 End point ID
     */
    void addLine(pcl::PointXYZ pt1, pcl::PointXYZ pt2, int id1, int id2);

 private:
    /**
     * PCL Visualizer class instance
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    /**
     * Transfers objects from queues to the internal visualizer class
     */
    void updateInternal();
    /**
     * Function run by worker class.
     */
    void spin();
    /**
     * Pointer to worker thread
     */
    boost::thread *viewer_thread;
    /**
     * Atomic flag. Used by stopSpin to stop worker thread
     */
    std::atomic_flag continueFlag = ATOMIC_FLAG_INIT;
    /**
     * Mutex to protect buffers
     */
    boost::mutex update_mutex;
    /**
     * Struct for cloud buffer
     */
    struct Cloud {
        PCLPointCloud cloud;
        int id;
    };
    /**
     * Struct for line buffer
     */
    struct Line {
        pcl::PointXYZ pt1, pt2;
        int id1, id2;
    };
    /**
     * queue for pointclouds
     */
    std::queue<Cloud> clouds;
    /**
     * queue for lines
     */
    std::queue<Line> lines;
};

}  // namespace wave

#endif  // WAVE_POINTCLOUDDISPLAY_HPP
