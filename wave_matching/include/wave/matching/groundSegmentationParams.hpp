#ifndef WAVE_GROUNDSEGMENTATIONPARAMS_H
#define WAVE_GROUNDSEGMENTATIONPARAMS_H

#include "wave/utils/config.hpp"
#include "wave/utils/log.hpp"

namespace wave {

struct GroundSegmentationParams {
    // variables
    double rmax;         // max radius of point to consider
    int max_bin_points;  // max number of points to consider per bin
    int num_seed_points;

    // for GP model
    float p_l;  // length parameter, so how close the points have to be in the
    // GP model to consider them correlated
    float p_sf;      // scaling on the whole covariance function
    float p_sn;      // the expected noise for the mode
    float p_tmodel;  // the required confidence required in order to consider
    // something ground
    float p_tdata;  // scaled value that is required for a query point to be
    // considered ground
    float p_tg;  // ground height threshold

    double robot_height;

    // seeding parameters
    double max_seed_range;   // meters
    double max_seed_height;  // meters

    int num_bins_a;
    int num_bins_l;

    // set default parameters
    GroundSegmentationParams() {
        rmax = 100;
        max_bin_points = 200;
        num_seed_points = 10;
        num_bins_a = 72;
        num_bins_l = 200;
        p_l = 10;
        p_sf = 1;
        p_sn = 0.3;
        p_tmodel = 5;
        p_tdata = 5;
        p_tg = 0.3;
        robot_height = 1.2;
        max_seed_range = 50;
        max_seed_height = 15;
    }

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
        if (parser.load(config_path) != 0) {
            LOG_ERROR("Unable to load config");
        }
    }
};

}  // namespace wave

#endif  // WAVE_GROUNDSEGMENTATIONPARAMS_H
