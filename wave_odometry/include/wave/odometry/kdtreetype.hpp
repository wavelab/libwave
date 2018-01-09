#ifndef WAVE_KDTREETYPE_HPP
#define WAVE_KDTREETYPE_HPP

#include <nanoflann.hpp>

namespace wave {

template<typename T>
struct FeatureKDTree {
    std::vector<std::array<T, 3>> points;

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
            return points[idx][0];
        else if (dim == 1)
            return points[idx][1];
        else
            return points[idx][2];
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

using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, FeatureKDTree<double>>,
        FeatureKDTree<double>,
        3>;

}

#endif //WAVE_KDTREETYPE_HPP
