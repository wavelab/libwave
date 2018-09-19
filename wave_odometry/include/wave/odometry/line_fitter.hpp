#ifndef WAVE_LINE_FITTER_HPP
#define WAVE_LINE_FITTER_HPP

#include <nabo/nabo.h>
#include <unsupported/Eigen/CXX11/Tensor>

#include "wave/odometry/feature_track.hpp"

namespace wave {

class LineFitter {
 private:
    ///params
    const float search_radius;
    const long search_knn;
    const double max_error;
    const int min_points;

    VecE<FeatureTrack> tracks;
 public:
    LineFitter(const float search_radius, const long search_knn, const double max_error, const int min_points)
            : search_radius(search_radius), search_knn(search_knn), max_error(max_error), min_points(min_points) {}

    void fitLines(const MatXf &candidate_points);

    auto &getTracks() {
        return this->tracks;
    }
};

}

#endif //WAVE_LINE_FITTER_HPP
