#ifndef WAVE_GROUNDSEGMENTATIONPARAMS_H
#define WAVE_GROUNDSEGMENTATIONPARAMS_H

#include "wave/utils/config.hpp"
#include "wave/utils/log.hpp"

namespace wave {

struct GroundSegmentationParams {
    // variables
    double rmax = 100;         // max radius of point to consider
    int max_bin_points = 200;  // max number of points to consider per bin
    int num_seed_points = 10;

    // for GP model
    float p_l = 4;  // length parameter, how close points have to be in the
    // GP model to correlate them
    float p_sf = 1;    // scaling on the whole covariance function
    float p_sn = 0.3;  // the expected noise for the mode
    float p_tmodel =
      5;  // the required confidence required in order to consider
    // something ground
    float p_tdata = 5;  // scaled value that is required for a query point to be
    // considered ground
    float p_tg = 0.3;  // ground height threshold

    double robot_height = 1.2;

    // seeding parameters
    double max_seed_range = 50;   // meters
    double max_seed_height = 15;  // meters

    int num_bins_a = 72;
    int num_bins_l = 200;

    // set default parameters
    GroundSegmentationParams() {}

    // use config file to set parameters
    GroundSegmentationParams(const std::string &config_path) {
        ConfigParser parser;
        parser.addParam("rmax", &rmax);
        parser.addParam("num_maxbinpoints", &max_bin_points);
        parser.addParam("num_seedpoints", &num_seed_points);
        parser.addParam("num_ang_bins", &num_bins_a);
        parser.addParam("num_lin_bins", &num_bins_l);
        parser.addParam("gp_lengthparameter", &p_l);
        parser.addParam("gp_covariancescale", &p_sf);
        parser.addParam("gp_modelnoise", &p_sn);
        parser.addParam("gp_groundmodelconfidence", &p_tmodel);
        parser.addParam("gp_grounddataconfidence", &p_tdata);
        parser.addParam("gp_groundthreshold", &p_tg);
        parser.addParam("robotheight", &robot_height);
        parser.addParam("seeding_maxrange", &max_seed_range);
        parser.addParam("seeding_maxheight", &max_seed_height);
        if (parser.load(config_path) != ConfigStatus::OK) {
            LOG_ERROR("Unable to load config");
        }
    }
};

}  // namespace wave

#endif  // WAVE_GROUNDSEGMENTATIONPARAMS_H
