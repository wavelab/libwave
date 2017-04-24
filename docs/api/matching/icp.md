# wave/matching/g_icp.hpp

This is a wrap of ICP in PCL

## Type Definitions

    namespace wave {

    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloud;

    }  // end of wave namespace

This type definition is used as a shorthand for the pointcloud object type used
by the scan matching implementations in PCL. A number of those implementations are
wrapped, so they use the same datatype for the pointcloud.

## GICP Matcher

    class GICPMatcher : public Matcher<PCLPointCloud> {
     public:
        explicit GICPMatcher(float resolution, const std::string& config_path);
        void setRef(const PCLPointCloud &ref);
        void setTarget(const PCLPointCloud &target);
        bool match();

     private:
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        PCLPointCloud ref, target, final;
    };


### Parameters

There are a few parameters that may be changed specific to this algorithm. They
can be set in the yaml config file.

* corr_rand: nearest neighbour correspondences used to calculate distributions
* max_iter: Limits number of ICP iterations
* t_eps: Criteria to stop iterating. If the difference between consecutive transformations is less than this, stop.
* fit_eps: Criteria to stop iterating. If the cost function does
not improve by more than this quantity, stop.

### Constructor

    explicit ICPMatcher(float resolution, const std::string& config_path);

This constructor takes an argument in order to adjust how much downsampling is
done before matching is attempted. Pointclouds are downsampled using a voxel filter,
the argument is the edge length of each voxel. If resolution is non-positive,
no downsampling is used.

config_path is the path to the yaml config file

### Private Members

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp

This is an instance of the G-ICP class from PCL

---

    pcl::VoxelGrid<pcl::PointXYZ> filter;

This is an instance of a PCL voxel filter. It is used to downsample input.

---

    PCLPointCloud ref, target, final;

These are pointers to the reference and target pointclouds. The "final" pointcloud
is not exposed. PCL's ICP class creates an aligned verison of the target pointcloud
after matching, so the "final" member is used as a sink for it.
