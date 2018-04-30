#include "wave/odometry/feature_extractor.hpp"

namespace wave {

void FeatureExtractor::init(unlong n_ring) {
    this->n_ring = n_ring;
    this->threadpool = std::unique_ptr<Eigen::ThreadPool>(new Eigen::ThreadPool(this->param.eigen_threads));
    this->thrddev = std::unique_ptr<Eigen::ThreadPoolDevice>(
      new Eigen::ThreadPoolDevice(this->threadpool.get(), this->param.eigen_threads));
}

void FeatureExtractor::setParams(FeatureExtractorParams params) {
    this->param = params;
}

void FeatureExtractor::computeScores(const Tensorf &signals, const Vec<int> &range) {
    Eigen::array<ptrdiff_t, 1> dims({1});
    Eigen::Tensor<float, 1> sum_kernel(this->param.variance_window);
    sum_kernel.setConstant(1.0);

    for (uint32_t i = 0; i < this->n_ring; i++) {
        auto max = (int) range.at(i);

        for (ulong j = 0; j < this->N_SCORES; j++) {
            // todo(ben) Include signal to score explicitly in feature definitions and get rid of this hack.
            int s_idx = 0;
            if (j < 1 || j == 3) {
                s_idx = 0;
            } else {
                s_idx = 1;
            }

            // todo have flexibility for different kernel sizes
            if (max > 10) {
                if (j < 3) {
                    this->scores.at(i).slice(ar2({(long int) j, 0}), ar2({1, max - 10})).device(*(this->thrddev)) =
                      signals.at(i).slice(ar2({s_idx, 0}), ar2({1, max})).convolve(this->kernels.at(j), dims);
                    // or if sample variance
                } else {
                    auto &N = this->param.variance_window;
                    Eigen::Tensor<float, 1> Ninv(1);
                    Ninv.setConstant(1.0 / (float) N);
                    Eigen::Tensor<float, 1> Nm1inv(1);
                    Nm1inv.setConstant(1.0 / (float) (N - 1));

                    // so called computational formula for sample variance
                    this->scores.at(i)
                      .slice(ar2({(int) j, 0}), ar2({1, max - (int) this->param.variance_window + 1}))
                      .device(*(this->thrddev)) =
                      (signals.at(i).slice(ar2({s_idx, 0}), ar2({1, max})).square().convolve(sum_kernel, dims) -
                       signals.at(i)
                         .slice(ar2({s_idx, 0}), ar2({1, max}))
                         .convolve(sum_kernel, dims)
                         .square()
                         .convolve(Ninv, dims))
                        .convolve(Nm1inv, dims);
                }
            }
        }
    }
}

void FeatureExtractor::preFilter(const Tensorf &scan, const Tensorf &signals, const Vec<int> &range) {
    for (uint32_t i = 0; i < this->n_ring; i++) {
        this->valid_pts.at(i) = Eigen::Tensor<bool, 1>(range.at(i));
        // Assume all points are valid until proven otherwise
        this->valid_pts.at(i).setConstant(true);

        Eigen::array<ptrdiff_t, 1> dims({1});
        Eigen::Tensor<float, 1> diff_kernel(2);
        diff_kernel.setValues({1.f, -1.f});

        Eigen::Tensor<float, 1> rng_diff;
        rng_diff.device(*(this->thrddev)) = signals.at(i).chip(0, 0).convolve(diff_kernel, dims);

        Eigen::Tensor<bool, 1> oc_tol2_cond = rng_diff.abs() > this->param.occlusion_tol_2;
        Eigen::Tensor<bool, 1> ang_diff_cond =
                scan.at(i).chip(4, 0).convolve(diff_kernel, dims) < this->param.occlusion_tol;

        Eigen::Tensor<bool, 1> branch_1_cond = oc_tol2_cond && ang_diff_cond && (rng_diff > 0.0f);
        Eigen::Tensor<bool, 1> branch_2_cond = oc_tol2_cond && ang_diff_cond && (rng_diff < 0.0f);

        // This section excludes any points whose nearby surface is
        // near to parallel to the laser beam
        Eigen::array<ptrdiff_t, 1> dims2({1});
        Eigen::Tensor<float, 2> l2diffkernel(3, 2);
        l2diffkernel.setValues({{1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, -1.0f}});

        Eigen::Tensor<float, 1> delforback;
        delforback.device(*(this->thrddev)) = scan.at(i)
                .slice(ar2({0, 0}), ar2({3, range.at(i)}))
                .convolve(l2diffkernel, dims2)
                .square()
                .sum(Earr<1>({0}));

        Eigen::Tensor<bool, 1> raw_cond =
                delforback > this->param.parallel_tol * signals.at(i).slice(ar2({0, 1}), ar2({1, range.at(i) - 1})).square();

        Eigen::Tensor<bool, 1> branch_3_cond;
        branch_3_cond.device(*(this->thrddev)) = raw_cond.slice(ar1({0}), ar1({(int) raw_cond.dimension(0) - 1})) &&
                                                raw_cond.slice(ar1({1}), ar1({(int) raw_cond.dimension(0) - 1}));

        Eigen::Tensor<bool, 1> false_tensor(range.at(i) - 2);
        false_tensor.setConstant(false);
        this->valid_pts.at(i).slice(ar1({1}), ar1({range.at(i) - 2})) =
                branch_3_cond.select(false_tensor, this->valid_pts.at(i).slice(ar1({1}), ar1({range.at(i) - 2})));

        for (int j = 0; j + 1 < range.at(i); j++) {
            if (branch_1_cond(j)) {
                int start;
                (j - 5) >= 0 ? start = j - 5 : start = 0;

                ar2 starts({start});
                ar2 extents({j - start});

                this->valid_pts.at(i).slice(starts, extents).setConstant(false);
            }
            if (branch_2_cond(j)) {
                int end;
                (j + 5) >= range.at(i) ? end = range.at(i) - 1 : end = j + 5;

                ar2 starts({j});
                ar2 extents({end - j});

                this->valid_pts.at(i).slice(starts, extents).setConstant(false);
            }
        }
    }
    // now store each selected point for sorting
    // Each point will be only be put in if it is a valid candidate for
    // that feature type
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
    for (uint32_t k = 0; k < this->N_FEATURES; k++) {
        auto &def = this->feature_definitions.at(k);
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
            switch (def.criteria.at(i).kernel) {
                case Kernel::LOAM: k_idx.at(i) = 0; break;
                case Kernel::LOG: k_idx.at(i) = 1; break;
                case Kernel::FOG: k_idx.at(i) = 2; break;
                case Kernel::RNG_VAR: k_idx.at(i) = 3; break;
                case Kernel::INT_VAR: k_idx.at(i) = 4; break;
                default: throw std::out_of_range("Unrecognized Kernel!");
            }
            // todo something about this
            k_offsets.emplace_back(5);
            offset = 5;
        }

        for (uint32_t i = 0; i < this->n_ring; i++) {
            this->filtered_scores.at(k).at(i).clear();
            Eigen::Tensor<bool, 1> condition = this->valid_pts.at(i).slice(ar1({offset}), ar1({range.at(i) - 2 * offset}));
            for (uint32_t l = 0; l < def.criteria.size(); l++) {
                condition = condition && compfuns.at(l)(this->scores.at(i).chip(k_idx.at(l), 0), *(def.criteria.at(l).threshold));
            }

            for (int j = offset; j + offset < condition.dimension(0); j++) {
                if (condition(j)) {
                    this->filtered_scores[k].at(i).emplace_back(j, this->scores.at(i)(k_idx.at(0), j - offset));
                }
            }
        }
    }
}

void FeatureExtractor::sortAndBin() {
    std::vector<unlong> cnt_in_bins;
    cnt_in_bins.resize(this->param.angular_bins);
    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
        this->feature_cnt(i, 0) = 0;
        for (unlong j = 0; j < this->param.n_ring; j++) {
            auto &def = this->feature_definitions.at(i);
            auto &pol = def.criteria.at(0).sel_pol;
            auto &filt_scores = this->filtered_scores.at(i).at(j);

            Eigen::Tensor<bool, 1> valid_pts_copy;
            valid_pts_copy = this->valid_pts.at(j);

            unlong max_bin = *(def.n_limit) / this->param.angular_bins;
            std::fill(cnt_in_bins.begin(), cnt_in_bins.end(), 0);

            if (pol == SelectionPolicy::HIGH_NEG || pol == SelectionPolicy::NEAR_ZERO) {
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

            for (auto score : filt_scores) {
                // Using data conversion to floor result
                unlong bin = (unlong) scan.at(j)(3, score.first) * this->param.angular_bins;
                if (cnt_in_bins.at(bin) >= max_bin) {
                    continue;
                }
                if (valid_pts_copy(score.first)) {
                    this->feature_points.at(i).slice(ar2({0, this->feature_cnt(i, 0)}), ar2({3, 1})) =
                            scan.at(j).slice(ar2({0, (int) score.first}), ar2({3, 1}));
                    this->flagNearbyPoints(score.first, valid_pts_copy);
                    cnt_in_bins.at(bin)++;
                    this->feature_cnt(i, 0)++;
                }
            }
        }
    }
}

void FeatureExtractor::getFeatures(const Tensorf &scan, const Tensorf &signals,
                                   const std::vector<int> &range, TensorIdx &indices) {
    this->computeScores(signals, range);
    this->prefilter(<#initializer#>, <#initializer#>);
    this->buildFilteredScore(<#initializer#>);
    this->sortAndBin();
}

}
