// This is to evaluate kitti sequences

#include "wave/matching/pointcloud_display.hpp"
#include "wave/odometry/laser_odom.hpp"
#include "wave/utils/config.hpp"

//todo Figure out where these helper functions should go (they are shared with tests).
namespace {

void LoadSensorParameters(const std::string &path, const std::string &filename, RangeSensorParams &senparams) {
    wave::ConfigParser parser;
    parser.addParam("rings", &(senparams.rings));
    parser.addParam("sigma_spherical", &(senparams.sigma_spherical));
    parser.addParam("elevation_angles", &(senparams.elevation_angles));

    parser.load(path + filename);
}

void LoadParameters(const std::string &path, const std::string &filename, wave::LaserOdomParams &params) {
    wave::ConfigParser parser;
    parser.addParam("Qc", &(params.Qc));
    parser.addParam("num_trajectory_states", &(params.num_trajectory_states));
    parser.addParam("n_window", &(params.n_window));
    parser.addParam("opt_iters", &(params.opt_iters));
    parser.addParam("max_inner_iters", &(params.max_inner_iters));
    parser.addParam("diff_tol", &(params.diff_tol));
    parser.addParam("solver_threads", &(params.solver_threads));
    parser.addParam("robust_param", &(params.robust_param));
    parser.addParam("max_correspondence_dist", &(params.max_correspondence_dist));
    parser.addParam("max_residual_val", &(params.max_residual_val));
    parser.addParam("min_residuals", &(params.min_residuals));
    parser.addParam("scan_period", &(params.scan_period));
    parser.addParam("max_ticks", &(params.max_ticks));
    parser.addParam("n_ring", &(params.n_ring));

    LoadSensorParameters(path, "sensor_model.yaml", params.sensor_params);

    parser.addParam("azimuth_tol", &(params.azimuth_tol));
    parser.addParam("max_planar_dist_threshold", &(params.max_planar_dist_threshold));
    parser.addParam("max_planar_ang_threshold", &(params.max_planar_ang_threshold));
    parser.addParam("max_linear_dist_threshold", &(params.max_linear_dist_threshold));
    parser.addParam("max_linear_ang_threshold", &(params.max_linear_ang_threshold));
    parser.addParam("ang_scaling_param", &(params.ang_scaling_param));

    parser.addParam("only_extract_features", &(params.only_extract_features));
    parser.addParam("use_weighting", &(params.use_weighting));
    parser.addParam("print_opt_sum", &(params.print_opt_sum));

    parser.load(path + filename);
}

void loadFeatureParams(const std::string &path, const std::string &filename, FeatureExtractorParams &params) {
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

    parser.load(path + filename);
}

void setupFeatureParameters(FeatureExtractorParams &param) {
    std::vector<Criteria> edge_high, edge_low, flat, edge_int_high, edge_int_low;
    edge_high.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::HIGH_POS, &(param.edge_tol)});

    edge_low.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::HIGH_NEG, &(param.edge_tol)});

    flat.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.flat_tol)});

    edge_int_high.emplace_back(
            Criteria{Signal::INTENSITY, Kernel::FOG, SelectionPolicy::HIGH_POS, &(param.int_edge_tol)});
    edge_int_high.emplace_back(
            Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_high.emplace_back(
            Criteria{Signal::RANGE, Kernel::RNG_VAR, SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    edge_int_low.emplace_back(
            Criteria{Signal::INTENSITY, Kernel::FOG, SelectionPolicy::HIGH_NEG, &(param.int_edge_tol)});
    edge_int_low.emplace_back(Criteria{Signal::RANGE, Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_low.emplace_back(
            Criteria{Signal::RANGE, Kernel::RNG_VAR, SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    param.feature_definitions.clear();
    param.feature_definitions.emplace_back(FeatureDefinition{edge_high, &(param.n_edge)});
    param.feature_definitions.emplace_back(FeatureDefinition{edge_low, &(param.n_edge)});
    param.feature_definitions.emplace_back(FeatureDefinition{flat, &(param.n_flat)});
    param.feature_definitions.emplace_back(FeatureDefinition{edge_int_high, &(param.n_int_edge)});
    param.feature_definitions.emplace_back(FeatureDefinition{edge_int_low, &(param.n_int_edge)});
}

}

namespace wave_examples {



}

int main() {
    wave::LaserOdom()
}