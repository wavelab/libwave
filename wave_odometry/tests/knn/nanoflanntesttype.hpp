#ifndef WAVE_NANOFLANNTESTTYPE_HPP
#define WAVE_NANOFLANNTESTTYPE_HPP

#include <Eigen/Eigen>
#include <nanoflann.hpp>

namespace wave {

template<typename T>
struct FeatureKDTree {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> points;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const {
        return points.size();
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const {
        if (dim == 0)
            return points(0, idx);
        else if (dim == 1)
            return points(1, idx);
        else
            return points(2, idx);
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in
    //   "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3
    //   for point clouds)
    template<class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const {
        return false;
    }
};

template<typename T>
using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<T, FeatureKDTree<T>>,
        FeatureKDTree<T>,
        3>;

}

#endif //WAVE_NANOFLANNTESTTYPE_HPP
