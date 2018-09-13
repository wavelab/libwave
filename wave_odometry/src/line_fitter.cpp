#include "wave/odometry/line_fitter.hpp"

namespace wave {

/**
 * Algorithm description:
 * 1. Find n nearest neighbours within x radius.
 * 2. For each point, is it is still unused,
 *   2a Find the average direction from the query point to all other points and use to initialize line feature
 *   2b Robustly fit a line to the set of query point + neighbour points
 *   2c If there are at least k points having error below tolerance, create feature track with those k points
 *   2d. Mark the query, and all of its neighbours as used.
 *
 * @param candidate_points input pointcloud
 */

namespace {

void calculateLineSimilarity(const wave::Vec6 &geo1,
                            const wave::Vec6 &geo2,
                            float &dist_cost,
                            float &dir_cost) {
    Vec3f diff = geo1.block<3, 1>(3, 0).cast<float>() - geo2.block<3, 1>(3, 0).cast<float>();
    dir_cost = 1.0f - std::abs((geo1.block<3, 1>(0, 0).transpose() * geo2.block<3, 1>(0, 0))(0));

    Vec3f cross = geo1.block<3, 1>(0, 0).cross(geo2.block<3, 1>(0, 0)).cast<float>();

    dist_cost = (diff.transpose() * cross).norm() / cross.norm();
}

bool compareMap(const wave::FeatureTrack::Mapping &lhs, const wave::FeatureTrack::Mapping &rhs) {
    return lhs.pt_idx < rhs.pt_idx;
}

void mergeTracks(wave::FeatureTrack &source, wave::FeatureTrack &sink) {
    std::sort(source.mapping.begin(), source.mapping.end(), compareMap);
    std::sort(sink.mapping.begin(), sink.mapping.end(), compareMap);
    for (uint32_t i = 0; i < source.mapping.size(); ++i) {
        auto iter = std::lower_bound(sink.mapping.begin(), sink.mapping.end(), source.mapping.at(i), compareMap);
        if (iter == sink.mapping.end()) {
            sink.mapping.emplace_back(source.mapping.at(i));
        } else if (iter->pt_idx == source.mapping.at(i).pt_idx) {
            continue;
        } else {
            sink.mapping.insert(iter, source.mapping.at(i));
        }
    }
    source.mapping.clear();
}

}

void LineFitter::fitLines(const MatXf &candidate_points) {
    if (candidate_points.cols() < 10) {
        return;
    }
    auto nn_search_tree = Nabo::NNSearchF::createKDTreeLinearHeap(candidate_points, 3);
    this->tracks.clear();

    long knn = search_knn < candidate_points.cols() - 1 ? search_knn : candidate_points.cols() - 1;

    Eigen::MatrixXi indices(knn, candidate_points.cols());
    MatXf dist(knn, candidate_points.cols());

    nn_search_tree->knn(candidate_points, indices, dist, knn, 0, Nabo::NNSearchF::SORT_RESULTS, this->search_radius);

    ceres::TinySolver<RobustLineFit> solver;

    for (long j = 0; j < candidate_points.cols(); ++j) {
        VecE<FeatureTrack> prototype_lines;
        for (long i = 0; i < knn; ++i) {
            if (std::isinf(dist(i, j))) {
                continue;
            }
            float a2 = dist(i, j);
            for (long k = 0; k < knn; ++k) {
                if (k == i) {
                    continue;
                }
                //see if indices(i,j) and indices(k,j) are within search radius of each other
                if (std::isinf(dist(k, j))) {
                    continue;
                }
                float b2 = dist(k, j);
                int ptA = indices(i,j);
                int ptB = indices(k,j);

                float c2 = (candidate_points.block<3,1>(0,ptA) - candidate_points.block<3,1>(0,ptB)).squaredNorm();

                float sum = a2 + b2 + c2;
                float area = 0.25 * std::sqrt(4.0*(a2*b2 + a2*c2 + b2*c2) - sum * sum);
                float largest = a2 > b2 ? (a2 > c2 ? a2 : c2) : (b2 > c2 ? b2 : c2);
                float error = 2 * area / std::sqrt(largest);
                if (error < this->max_error) {
                    FeatureTrack new_track;
                    new_track.geometry.block<3, 1>(0, 0) = (candidate_points.block<3, 1>(0, indices(i, j)) -
                                                            candidate_points.block<3, 1>(0, indices(k,
                                                                                                    j))).cast<double>();
                    new_track.geometry.block<3, 1>(0, 0).normalize();
                    new_track.geometry.block<3, 1>(3, 0) = 0.5 * (candidate_points.block<3, 1>(0, indices(i, j)) +
                                                                  candidate_points.block<3, 1>(0, indices(k,
                                                                                                          j))).cast<double>();
                    new_track.mapping.emplace_back(j, 0);
                    new_track.mapping.emplace_back(indices(i, j), 0);
                    new_track.mapping.emplace_back(indices(k, j), 0);
                    prototype_lines.emplace_back(new_track);
                }
            }
        }
        // protoype lines should now contain all of the feasible lines having at least 3 points within the search radius
        // now. Every track contains the query point, and at least one unique point.
        if (!prototype_lines.empty()) {
            for (uint32_t i = 0; i < prototype_lines.size(); ++i) {
                auto &first_line = prototype_lines.at(i);
                for (uint32_t m = i + 1; m < prototype_lines.size(); ++m) {
                    auto &second_line = prototype_lines.at(m);
                    float dir_cost, dist_cost;
                    calculateLineSimilarity(first_line.geometry, second_line.geometry, dist_cost, dir_cost);
                    if (dist_cost < this->max_error && dir_cost < 0.004) {
                        mergeTracks(first_line, second_line);
                    }
                }
            }
            std::sort(prototype_lines.begin(), prototype_lines.end(), [](const FeatureTrack &lhs, const FeatureTrack &rhs) {
                return lhs.mapping.size() < rhs.mapping.size();
            });
            if (prototype_lines.back().mapping.size() >= static_cast<size_t>(this->min_points)) {
                this->tracks.emplace_back(prototype_lines.back());
            }
        }
    }
}

}
