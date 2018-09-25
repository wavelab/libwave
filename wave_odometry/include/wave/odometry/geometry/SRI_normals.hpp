// Class to calculate point normals using a spherical range image
// Input data should contain nans or infs when data is missing

#ifndef WAVE_SRI_NORMALS_HPP
#define WAVE_SRI_NORMALS_HPP

#include <unsupported/Eigen/CXX11/Tensor>
#include <nabo/nabo.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Triangulation_vertex_base_with_info_2.h>
//#include <CGAL/Triangulation_hierarchy_2.h>
//#include <CGAL/Delaunay_triangulation_2.h>
#include "wave/odometry/odometry_types.hpp"

namespace wave {

struct NormalEstimatorParams {
    double min_elevation = -0.453785606; // -26 degrees
    double max_elevation = 0.27925268; // 16 degrees
    double azimuth_range = 6.457718232; // slightly more than one revolution
    double azimuth_resolution = 0.001919862; // 0.11 degrees
    double elevation_resolution = 0.003490659; // 0.2 degrees
};

class NormalEstimator {
 public:
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Vbb = CGAL::Triangulation_vertex_base_with_info_2<float, K>;
    using Vb = CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>;
    using Fb = CGAL::Triangulation_face_base_2<K>;
    using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using Triangulation = CGAL::Delaunay_triangulation_2<K, Tds>;
//    using Triangulation = CGAL::Triangulation_hierarchy_2<Dt>;

    NormalEstimator(NormalEstimatorParams param);

    /**
    * This takes a flat input scan and prepares the masks
    * @param input_scan Eigen tensor, k x N where k is at least 3 and the first three rows hold azimuth, elevation, and range
    * in that order and N is the number of points to form the image on.
    */
    void setInput(const Eigen::Tensor<float, 2> &input_scan, long n_rings);
    void interpolate();

    //getters for testing
    const Eigen::Tensor<bool, 2>& getNormalMask();
    const Eigen::Tensor<bool, 2>& getInterpolationMask();
    const Eigen::Tensor<float, 2>& getInterpolatedImage();
 private:
    NormalEstimatorParams param;
    Eigen::Tensor<bool, 2> normal_mask;
    Eigen::Tensor<bool, 2> interpolation_mask;
    Eigen::Tensor<float, 2> interpolated_image;
    float starting_azimuth;

    Vec<std::pair<Triangulation::Point, float>> points;
    Triangulation triangulation;

    float getInterpolatedRange(const Triangulation::Point &pt, const Triangulation::Face_handle &fh);
    void calculateGradients();
    void calculateNormals();
};

}

#endif //WAVE_SRI_NORMALS_HPP
