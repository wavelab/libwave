/** @file
 * @ingroup matching
 *
 * Interface to PCL Visualiser Class
 *
 * This is a class that provides an interface to PCLVisualiser class for
 * making cool graphics and debugging. The visualiser itself runs in a
 * separate thread so that it is always interactive.
 * Note: Ids are shared between all drawn objects, so they should be
 * globally unique.
 */

#ifndef WAVE_POINTCLOUDDISPLAY_HPP
#define WAVE_POINTCLOUDDISPLAY_HPP

#include <atomic>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "wave/matching/pcl_common.hpp"

namespace wave {
/** @addtogroup matching
 *  @{ */

class PointCloudDisplay {
 public:
    /**
     * Default constructor
     * @param name: Display window title. Must be unique.
     */
    PointCloudDisplay(const std::string &name, double radius = 0.2);

    ~PointCloudDisplay();

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
     *
     * A copy of the cloud is taken. To show changes, call addPointcloud again
     * with the same id.
     *
     * @param cld: cloud to be added
     * @param id: identifier of cloud to be added. The id can be used to update
     * the cloud later.
     * @param reset_camera if true, moves the camera to fit the scene
     */

    void addPointcloud(const PCLPointCloudPtr &cld,
                       int id,
                       bool reset_camera = false);

    void addPointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cld,
                       int id,
                       bool reset_camera = false);

    /**
     * Queues a line to be added to the display.
     * Note that three objects are added for one line:
     * spheres at the endpoints, and a line between them.
     * If worker thread isn't running nothing will happen.
     * @param pt1 Start coordinate of line
     * @param pt2 End coordinate of line
     * @param id1 Start point ID
     * @param id2 End point ID
     * @param reset_camera if true, moves the camera to fit the scene
     */
    void addLine(const pcl::PointXYZ &pt1,
                 const pcl::PointXYZ &pt2,
                 int id1,
                 int id2,
                 bool reset_camera = false);

    void removeAll();

 private:
    std::string display_name;
    /**
     * PCL Visualizer class instance
     */
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    /**
     * Transfers objects from queues to the internal visualizer class
     */
    void updateInternal();
    /**
     * Function run by worker class.
     */
    void spin();

    std::thread viewer_thread;

    /**
     * Used to stop worker thread
     */
    std::atomic_flag continueFlag = ATOMIC_FLAG_INIT;
    std::atomic_flag resetLines = ATOMIC_FLAG_INIT;
    std::mutex update_mutex;  ///< Protects buffers
    /** Struct for cloud buffer */
    struct Cloud {
        PCLPointCloudPtr cloud;
        int id;
        bool reset_camera;
    };

    struct CloudI {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        int id;
        bool reset_camera;
    };

    /** Struct for line buffer */
    struct Line {
        pcl::PointXYZ pt1, pt2;
        int id1, id2;
        bool reset_camera;
    };

    std::queue<Cloud> clouds;
    std::queue<CloudI> cloudsi;
    std::queue<Line> lines;
    double radius;  // radius for line endpoints.
};

}  // namespace wave

#endif  // WAVE_POINTCLOUDDISPLAY_HPP
