#include "wave/odometry/feature_extractor.hpp"

namespace wave {

void FeatureExtractor::setup() {
    ///setup all the kernel-signal combinations
    for (const auto &def : this->param.feature_definitions) {
        for (const auto &crit : def.criteria) {
            auto combo = std::make_pair(crit.kernel, crit.signal);
            if (std::find(this->proc_vec.begin(), this->proc_vec.end(), combo) == this->proc_vec.end()) {
                this->proc_vec.emplace_back(combo);
            }
        }
    }

    this->valid_pts.resize(this->n_ring);
    this->scores.resize(this->n_ring);

    for (uint32_t i = 0; i < this->scores.size(); ++i) {
        for (const auto &proc_def : this->proc_vec) {
            this->scores.at(i).emplace(proc_def, Eigen::Tensor<float, 1>());
        }
    }

    this->filtered_scores.resize(this->param.N_SCORES);
    for (uint32_t i = 0; i < this->param.N_SCORES; i++) {
        this->filtered_scores.at(i).resize(this->n_ring);
    }

    Eigen::Tensor<float, 1> temporary(11);
    temporary.setValues({1, 1, 1, 1, 1, -10, 1, 1, 1, 1, 1});
    this->kernels.emplace(Kernel::LOAM, temporary);
    temporary.setValues({0.000232391821040f,
                         0.001842097682135f,
                         0.034270489647107f,
                         0.166944943945706f,
                         -0.009954755288609f,
                         -0.386583206270711f,
                         -0.009954755288609f,
                         0.166944943945706f,
                         0.034270489647107f,
                         0.001842097682135f,
                         0.000232391821040f});
    this->kernels.emplace(Kernel::LOG, temporary);
//    temporary.setValues({0, 0.003571428, -0.0380952f, 0.2, -0.8f, 0, 0.8, -0.2f, 0.0380952, -0.003571428f, 0});
    temporary.setValues({-0.2f, -0.4f, -0.6f, -0.8f, -1.f, 0, 1.f, .8f, .6f, .4f, .2f});
    this->kernels.emplace(Kernel::FOG, temporary);
    temporary.setConstant(1.0);
    this->kernels.emplace(Kernel::VAR, temporary);

    this->ready = true;
}

FeatureExtractor::FeatureExtractor(FeatureExtractorParams params, unlong n_ring) : param(params), n_ring(n_ring) {
    this->setup();
}

void FeatureExtractor::setParams(FeatureExtractorParams params, unlong n_ring) {
    this->param = std::move(params);
    this->n_ring = n_ring;

    if (!this->ready) {
        this->setup();
    }
}

void FeatureExtractor::computeScores(const SignalVec &signals, const Vec<int> &range) {
    if (!this->ready) {
        throw std::length_error("Must set feature parameters before using");
    }
    for (uint32_t i = 0; i < this->n_ring; i++) {
        Eigen::array<ptrdiff_t, 1> dims({0});
        Eigen::Tensor<float, 1> sum_kernel(this->param.variance_window);
        sum_kernel.setConstant(1.0);

        auto max = (int) range.at(i);
        // resize
        if (max < 11) {
            continue;
        }

        for (const auto &proc_pair : this->proc_vec) {
            const auto &kernel = this->kernels.at(std::get<0>(proc_pair));
            auto &signal = signals.at(i).at(std::get<1>(proc_pair));
            auto &score = this->scores.at(i).at(proc_pair);
            score = Eigen::Tensor<float, 1>(max - 10);

            Eigen::Tensor<float, 1> cur_signal = signal.slice(ar1({0}), ar1({max}));

            // todo have flexibility for different kernel sizes
            if (std::get<0>(proc_pair) != Kernel::VAR) {
                score = cur_signal.convolve(kernel, dims);
                // or if sample variance
            } else {
                auto &N = this->param.variance_window;
                Eigen::Tensor<float, 1> Ninv(1);
                Ninv.setConstant(1.0 / (float) N);
                Eigen::Tensor<float, 1> Nm1inv(1);
                Nm1inv.setConstant(1.0 / (float) (N - 1));

                // so called computational formula for sample variance
                // todo. This is quite expensive, should only calculate sample variance on high/low scores
                score =
                  (cur_signal.square().convolve(sum_kernel, dims) -
                          cur_signal
                     .convolve(sum_kernel, dims)
                     .square()
                     .convolve(Ninv, dims))
                    .convolve(Nm1inv, dims);
            }
        }
    }
}

void FeatureExtractor::preFilter(const Tensorf &scan, const SignalVec &signals, const Vec<int> &true_size) {
//    Eigen::ThreadPool tp(4);
//    Eigen::ThreadPoolDevice device(&tp, 4);
    for (uint32_t i = 0; i < this->n_ring; i++) {
        if (true_size.at(i) < 11) {
            this->valid_pts.at(i).resize(0);
            continue;
        }
        this->valid_pts.at(i).resize(true_size.at(i));
        // Assume all points are valid until proven otherwise
        this->valid_pts.at(i).setConstant(true);

        // This first chunk removes points next to a shadow
        Eigen::Tensor<float, 1> range = signals.at(i).at(Signal::RANGE).slice(ar1{0}, ar1{true_size.at(i)});
        Eigen::Tensor<float, 1> azimuths = scan.at(i).slice(ar2{4,0}, ar2{1,true_size.at(i)}).chip(0,0);
        Eigen::Tensor<float, 2> cur_scan = scan.at(i).slice(ar2{0,0}, ar2{3,true_size.at(i)});

        Eigen::Tensor<float, 1> diff_kernel(2);
        diff_kernel.setValues({1.f, -1.f});
        Eigen::Tensor<float, 1> rng_diff(true_size.at(i) - 1);
        rng_diff = range.convolve(diff_kernel, ar1({0}));
        Eigen::Tensor<bool, 1> oc_tol2_cond = rng_diff.abs() > this->param.occlusion_tol_2;

        Eigen::Tensor<bool, 1> ang_diff_cond(true_size.at(i) - 1);
        Eigen::Tensor<float, 1> delta_theta;
        delta_theta = azimuths.slice(ar1{0}, ar1{true_size.at(i) - 1}) - azimuths.slice(ar1{1}, ar1{true_size.at(i) - 1});

        Eigen::Tensor<bool, 1> too_high = delta_theta > (float) M_PI;
        Eigen::Tensor<bool, 1> too_low = delta_theta < (float) -M_PI;
        delta_theta = too_high.select(delta_theta - (float)(2*M_PI), too_low.select(delta_theta + (float)(2*M_PI), delta_theta));

        ang_diff_cond = delta_theta > this->param.occlusion_tol;

        Eigen::Tensor<bool, 1> branch_1_cond(true_size.at(i) - 1);
        Eigen::Tensor<bool, 1> branch_2_cond(true_size.at(i) - 1);
        branch_1_cond = oc_tol2_cond && ang_diff_cond && (rng_diff > 0.0f);
        branch_2_cond = oc_tol2_cond && ang_diff_cond && (rng_diff < 0.0f);

        for (int j = 1; j + 1 < true_size.at(i); j++) {
            if (branch_1_cond(j)) {
                int start;
                (j - this->param.occlusion_filter_length) >= 0 ? start = j - this->param.occlusion_filter_length
                                                               : start = 0;

                ar1 starts({start});
                ar1 extents({j - start + 1});

                this->valid_pts.at(i).slice(starts, extents).setConstant(false);
            }
            if (branch_2_cond(j)) {
                int end;
                (j + 1 + this->param.occlusion_filter_length) >= true_size.at(i)
                  ? end = true_size.at(i) - 1
                  : end = j + 1 + this->param.occlusion_filter_length;

                int start = end - this->param.occlusion_filter_length;
                if (start < 0) {
                    start = 0;
                }
                ar1 starts({start});
                ar1 extents({end - start});

                this->valid_pts.at(i).slice(starts, extents).setConstant(false);
            }
        }

//        // This section excludes any points whose nearby surface is
//        // near to parallel to the laser beam
//        Eigen::array<ptrdiff_t, 2> dims2({0, 1});
//        Eigen::Tensor<float, 2> ex_diff_K(1, 2);
//        ex_diff_K.setValues({{1.f, -1.f}});
//
//        Eigen::Tensor<float, 1> delforback(true_size.at(i) - 1);
//
//        // This is the squared distance from each point in the scan to its neighbour
//        delforback =
//                cur_scan.convolve(ex_diff_K, dims2).eval().square().sum(Earr<1>({0}));
//
//        Eigen::Tensor<float, 1> sqr_rng(true_size.at(i) - 2);
//        sqr_rng = range.slice(ar1({1}), ar1({true_size.at(i) - 2})).square();
//
//        Eigen::Tensor<bool, 1> low_side_cond(true_size.at(i) - 2);
//        low_side_cond = delforback.slice(ar1({0}), ar1({true_size.at(i) - 2})) > this->param.parallel_tol * sqr_rng;
//        Eigen::Tensor<bool, 1> high_side_cond(true_size.at(i) - 2);
//        high_side_cond = delforback.slice(ar1({1}), ar1({true_size.at(i) - 2})) > this->param.parallel_tol * sqr_rng;
//
//        Eigen::Tensor<bool, 1> branch_3_cond(true_size.at(i) - 2);
//        branch_3_cond = low_side_cond && high_side_cond;
//
//        Eigen::Tensor<bool, 1> false_tensor(true_size.at(i) - 2);
//        false_tensor.setConstant(false);
//        this->valid_pts.at(i).slice(ar1({1}), ar1({true_size.at(i) - 2})) =
//                branch_3_cond.select(false_tensor, this->valid_pts.at(i).slice(ar1({1}), ar1({true_size.at(i) - 2})));
    }
}

namespace {

Eigen::Tensor<bool, 1> near_zero_score(const Eigen::Tensor<float, 1> &score, float threshold) {
    return score.abs() < threshold;
}

Eigen::Tensor<bool, 1> high_pos_score(const Eigen::Tensor<float, 1> &score, float threshold) {
    return score > threshold;
}

Eigen::Tensor<bool, 1> high_neg_score(const Eigen::Tensor<float, 1> &score, float threshold) {
    return score < -threshold;
}
}

void FeatureExtractor::buildFilteredScore(const Vec<int> &range) {
//#pragma omp parallel for
    for (int k = this->param.feature_definitions.size() - 1; k >= 0; k--) {
        auto &def = this->param.feature_definitions.at(k);
        // get primary score index by kernel type
        std::vector<std::function<Eigen::Tensor<bool, 1>(const Eigen::Tensor<float, 1> &, double)>> compfuns;
        std::vector<uint32_t> k_idx;
        std::vector<uint32_t> k_offsets;
        int offset = 0;
        compfuns.resize(def.criteria.size());
        k_idx.resize(def.criteria.size());
        for (uint32_t i = 0; i < def.criteria.size(); i++) {
            switch (def.criteria.at(i).sel_pol) {
                case SelectionPolicy::NEAR_ZERO: compfuns.at(i) = near_zero_score; break;
                case SelectionPolicy::HIGH_POS: compfuns.at(i) = high_pos_score; break;
                case SelectionPolicy::HIGH_NEG: compfuns.at(i) = high_neg_score; break;
                default: throw std::out_of_range("Invalid Comparison Function");
            }
            k_idx.at(i) = static_cast<uint32_t>(def.criteria.at(i).kernel);
            // todo something about this
            k_offsets.emplace_back(5);
            offset = 5;
        }

        for (uint32_t i = 0; i < this->n_ring; i++) {
            this->filtered_scores.at(k).at(i).clear();
            if (range.at(i) < 11) {
                continue;
            }
            Eigen::Tensor<bool, 1> condition =
              this->valid_pts.at(i).slice(ar1({offset}), ar1({range.at(i) - 2 * offset}));
            for (uint32_t l = 0; l < def.criteria.size(); l++) {
                auto combo = std::make_pair(def.criteria.at(l).kernel, def.criteria.at(l).signal);
                const auto &score = this->scores.at(i).at(combo);
                condition =
                  condition && compfuns.at(l)(score, *(def.criteria.at(l).threshold));
            }

            auto sort_combo = std::make_pair(def.criteria.front().kernel, def.criteria.front().signal);
            for (int j = offset; j + offset < condition.dimension(0); j++) {
                if (condition(j)) {
                    this->filtered_scores[k].at(i).emplace_back(j, scores.at(i).at(sort_combo)(j - offset));
                }
            }
        }
    }
}

void FeatureExtractor::flagNearbyPoints(const uint32_t p_idx, const float pt_range, Eigen::Tensor<bool, 1> &valid) {
    // key radius is specified to be at a range of 10 meters. That is the magic number here.
    auto scaled_key_radius = static_cast<uint32_t>(this->param.key_radius * 10.0f / pt_range);
    for (uint32_t j = 0; j < scaled_key_radius; j++) {
        if (p_idx + j + 1 >= valid.dimension(0)) {
            break;
        }
        valid(p_idx + j + 1) = false;
    }
    for (uint32_t j = 0; j < scaled_key_radius; j++) {
        if (p_idx < j + 1) {
            break;
        }
        valid(p_idx - j - 1) = false;
    }
}

void FeatureExtractor::sortAndBin(const Tensorf &scan, const SignalVec &signals, TensorIdx &feature_indices) {
//    #pragma omp parallel for
    for (uint32_t i = 0; i < this->param.feature_definitions.size(); i++) {
        for (unlong j = 0; j < this->n_ring; j++) {
            auto &def = this->param.feature_definitions.at(i);
            auto &pol = def.criteria.at(0).sel_pol;
            auto &filt_scores = this->filtered_scores.at(i).at(j);

            // feature_indices.at(i).at(j) = Eigen::Tensor<int, 1>(*(def.n_limit));
            Eigen::Tensor<int, 1> cur_feat_idx(*(def.n_limit));
            uint64_t feat_cnt = 0;

            Eigen::Tensor<bool, 1> valid_pts_copy;
            valid_pts_copy = this->valid_pts.at(j);

            unlong max_bin = *(def.n_limit) / this->param.angular_bins;
            std::vector<unlong> cnt_in_bins;
            cnt_in_bins.resize(this->param.angular_bins);
            std::fill(cnt_in_bins.begin(), cnt_in_bins.end(), 0);

            if (pol != SelectionPolicy::HIGH_POS) {
                std::sort(filt_scores.begin(),
                          filt_scores.end(),
                          [](const std::pair<unlong, double> lhs, const std::pair<unlong, double> rhs) {
                              return lhs.second < rhs.second;
                          });
            } else {
                std::sort(filt_scores.begin(),
                          filt_scores.end(),
                          [](const std::pair<unlong, double> lhs, const std::pair<unlong, double> rhs) {
                              return lhs.second > rhs.second;
                          });
            }

            const auto &range_signal = signals.at(j).at(Signal::RANGE);
            for (const auto &score : filt_scores) {
                // Using data conversion to floor result
                auto bin = (unlong)((scan.at(j)(4, score.first) / (2*M_PI)) * this->param.angular_bins);
                if (cnt_in_bins.at(bin) >= max_bin) {
                    continue;
                }
                if (valid_pts_copy(score.first)) {
                    cur_feat_idx(feat_cnt) = (int) score.first;
                    feat_cnt++;

                    this->flagNearbyPoints(score.first, range_signal(score.first), valid_pts_copy);
                    cnt_in_bins.at(bin)++;
                }
            }
            feature_indices.at(i).at(j) = cur_feat_idx.slice(ar1({0}), ar1({static_cast<long>(feat_cnt)}));
        }
    }
}

void FeatureExtractor::getFeatures(const Tensorf &scan,
                                   const SignalVec &signals,
                                   const std::vector<int> &range,
                                   TensorIdx &indices) {
    if (scan.size() != this->n_ring || signals.size() != this->n_ring) {
        throw std::length_error("mismatch between configured ring count and input scan/signals size");
    }
    this->computeScores(signals, range);
    this->preFilter(scan, signals, range);
    this->buildFilteredScore(range);
    this->sortAndBin(scan, signals, indices);
}

}
