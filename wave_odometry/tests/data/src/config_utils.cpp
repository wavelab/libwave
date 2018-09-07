#include "../include/config_utils.hpp"

namespace wave {

void LoadSensorParameters(const std::string &path, const std::string &filename, wave::RangeSensorParams &senparams) {
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
    parser.addParam("max_planar_residual_val", &(params.max_planar_residual_val));
    parser.addParam("max_linear_residual_val", &(params.max_linear_residual_val));
    parser.addParam("min_residuals", &(params.min_residuals));
    parser.addParam("n_ring", &(params.n_ring));

    LoadSensorParameters(path, "sensor_model.yaml", params.sensor_params);

    parser.addParam("elevation_tol", &(params.elevation_tol));
    parser.addParam("max_planar_dist_threshold", &(params.max_planar_dist_threshold));
    parser.addParam("max_planar_ang_threshold", &(params.max_planar_ang_threshold));
    parser.addParam("max_linear_dist_threshold", &(params.max_linear_dist_threshold));
    parser.addParam("max_linear_ang_threshold", &(params.max_linear_ang_threshold));
    parser.addParam("ang_scaling_param", &(params.ang_scaling_param));
    parser.addParam("min_new_points", &(params.min_new_points));

    parser.addParam("only_extract_features", &(params.only_extract_features));
    parser.addParam("use_weighting", &(params.use_weighting));
    parser.addParam("print_opt_sum", &(params.print_opt_sum));

    parser.load(path + filename);

    params.inv_Qc = params.Qc.inverse();
}

void loadFeatureParams(const std::string &path, const std::string &filename, wave::FeatureExtractorParams &params) {
    wave::ConfigParser parser;

    parser.addParam("variance_window", &(params.variance_window));
    parser.addParam("variance_limit_rng", &(params.variance_limit_rng));
    parser.addParam("variance_limit_int", &(params.variance_limit_int));
    parser.addParam("angular_bins", &(params.angular_bins));
    parser.addParam("min_intensity", &(params.min_intensity));
    parser.addParam("max_intensity", &(params.max_intensity));
    parser.addParam("occlusion_tol", &(params.occlusion_tol));
    parser.addParam("occlusion_tol_2", &(params.occlusion_tol_2));
    parser.addParam("occlusion_filter_length", &(params.occlusion_filter_length));
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

void setupFeatureParameters(wave::FeatureExtractorParams &param) {
    std::vector<wave::Criteria> edge_high, edge_low, flat, edge_int_high, edge_int_low;
    edge_high.emplace_back(
            wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::HIGH_POS, &(param.edge_tol)});

    edge_low.emplace_back(
            wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::HIGH_NEG, &(param.edge_tol)});

    flat.emplace_back(
            wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::NEAR_ZERO, &(param.flat_tol)});
    flat.emplace_back(wave::Criteria{
            wave::Signal::RANGE, wave::Kernel::VAR, wave::SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    edge_int_high.emplace_back(wave::Criteria{
            wave::Signal::INTENSITY, wave::Kernel::FOG, wave::SelectionPolicy::HIGH_POS, &(param.int_edge_tol)});
    edge_int_high.emplace_back(
            wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_high.emplace_back(wave::Criteria{
            wave::Signal::RANGE, wave::Kernel::VAR, wave::SelectionPolicy::NEAR_ZERO, &(param.variance_limit_int)});

    edge_int_low.emplace_back(wave::Criteria{
            wave::Signal::INTENSITY, wave::Kernel::FOG, wave::SelectionPolicy::HIGH_NEG, &(param.int_edge_tol)});
    edge_int_low.emplace_back(
            wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_low.emplace_back(wave::Criteria{
            wave::Signal::RANGE, wave::Kernel::VAR, wave::SelectionPolicy::NEAR_ZERO, &(param.variance_limit_int)});

    param.feature_definitions.clear();
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_high, &(param.n_edge)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_low, &(param.n_edge)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{flat, &(param.n_flat)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_int_high, &(param.n_int_edge)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_int_low, &(param.n_int_edge)});
}

void loadBinnerParams(const std::string &path, const std::string &filename, wave::IcosahedronBinnerParams &params) {
    wave::ConfigParser parser;

    parser.addParam("range_divisions", &(params.range_divisions));
    parser.addParam("azimuth_divisions", &(params.azimuth_divisions));
    parser.addParam("xy_directions", &(params.xy_directions));
    parser.addParam("z_cutoff", &(params.z_cutoff));
    parser.addParam("z_limit", &(params.z_limit));
    parser.addParam("xy_limit", &(params.xy_limit));

    parser.load(path + filename);
}

}
