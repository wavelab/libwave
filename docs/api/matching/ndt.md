# wave/matching/ndt.hpp

This is a wrap of NDT in PCL

## Type Definitions

    namespace wave {

    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloud;

    }  // end of wave namespace

This type definition is used as a shorthand for the pointcloud object type used
by the scan matching implementations in PCL. A number of those implementations are
wrapped, so they use the same datatype for the pointcloud.

## ICP Matcher

    class NDTMatcher : public Matcher<PCLPointCloud> {
     public:
        explicit NDTMatcher(float resolution, const std::string &config_path);
        void setRef(const PCLPointCloud &ref);
        void setTarget(const PCLPointCloud &target);
        bool match();

     private:
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
        PCLPointCloud ref, target, final;
        const float min_res = 0.05f;
    };


### Parameters

There are a few parameters that may be changed specific to this algorithm. They
can be set in the yaml config file.

* step_size: Maximum Newton step size used in line search
* max_iter: Limits number of iterations
* t_eps: Criteria to stop iterating. If the difference between consecutive transformations is less than this, stop.
* config_res: If the contructor is given an invalid resolution (too fine or negative), use this resolution

### Constructor

    explicit NDTMatcher(float resolution, const std::string &config_path);

This constructor takes an argument in order to specify resolution. The resolution
given takes precedence over the one in the config file. If both resolutions are
finer than the min_res class member, the resolution is set to min_res.

config_path is the path to the yaml config file

### Private Members

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

This is an instance of the NDT class from PCL

---

    PCLPointCloud ref, target, final;

These are pointers to the reference and target pointclouds. The "final" pointcloud
is not exposed. PCL's NDT class creates an aligned verison of the target pointcloud
after matching, so the "final" member is used as a sink for it.
