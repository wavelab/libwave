#define EIGEN_NO_MALLOC 1

#include "wave/wave_test.hpp"
#include "wave/odometry/feature_extractor.hpp"

namespace wave{

namespace {

void LoadParameters(const std::string &path, const std::string &filename, FeatureExtractorParams &params) {
    wave::ConfigParser parser;

    parser.addParam("variance_window", &(params.variance_window));
    parser.addParam("variance_limit_rng", &(params.variance_limit_rng));
    parser.addParam("variance_limit_int", &(params.variance_limit_int));
    parser.addParam("angular_bins", &(params.angular_bins));
    parser.addParam("min_intensity", &(params.min_intensity));
    parser.addParam("max_intensity", &(params.max_intensity));
    parser.addParam("occlusion_tol", &(params.occlusion_tol));
    parser.addParam("occlusion_tol_2", &(params.occlusion_tol_2));
    parser.addParam("parallel_tol", &(params.parallel_tol));
    parser.addParam("edge_tol", &(params.edge_tol));
    parser.addParam("flat_tol", &(params.flat_tol));
    parser.addParam("int_edge_tol", &(params.int_edge_tol));
    parser.addParam("int_flat_tol", &(params.int_flat_tol));
    parser.addParam("n_edge", &(params.n_edge));
    parser.addParam("n_flat", &(params.n_flat));
    parser.addParam("n_int_edge", &(params.n_int_edge));
    parser.addParam("knn", &(params.knn));
    parser.addParam("key_radius", &(params.key_radius));
    parser.addParam("edge_map_density", &(params.edge_map_density));
    parser.addParam("flat_map_density", &(params.flat_map_density));
    parser.addParam("azimuth_tol", &(params.azimuth_tol));
    parser.addParam("TTL", &(params.TTL));
    parser.addParam("min_eigen", &(params.min_eigen));
    parser.addParam("max_extrapolation", &(params.max_extrapolation));
    parser.addParam("visualize", &(params.visualize));
    parser.addParam("output_trajectory", &(params.output_trajectory));
    parser.addParam("output_correspondences", &(params.output_correspondences));
    parser.addParam("only_extract_features", &(params.only_extract_features));
    parser.addParam("use_weighting", &(params.use_weighting));
    parser.addParam("lock_first", &(params.lock_first));
    parser.addParam("plot_stuff", &(params.plot_stuff));
    parser.addParam("solution_remapping", &(params.solution_remapping));
    parser.addParam("motion_prior", &(params.motion_prior));
    parser.addParam("no_extrapolation", &(params.no_extrapolation));
    parser.addParam("treat_lines_as_planes", &(params.treat_lines_as_planes));

    parser.load(path + filename);
}

}

}
