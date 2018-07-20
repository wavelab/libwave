#include "wave/odometry/laser_odom.hpp"

namespace wave {

LaserOdom::LaserOdom(const LaserOdomParams params, const FeatureExtractorParams feat_params)
    : param(params), feature_extractor(feat_params, params.n_ring), transformer(Transformer(TransformerParams())) {
    this->CSVFormat = new Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ");

    auto n_ring = static_cast<size_t>(param.n_ring);
    this->feature_extractor.setParams(feat_params, n_ring);
    this->counters.resize(n_ring);
    std::fill(this->counters.begin(), this->counters.end(), 0);

    this->cur_feature_candidates.resize(this->N_FEATURES);
    this->cur_feature_candidatesT.resize(this->N_FEATURES);
    this->prev_feature_candidates.resize(this->N_FEATURES);

    this->cur_scan.resize(n_ring);
    this->signals.resize(n_ring);

    for (uint32_t i = 0; i < n_ring; i++) {
        this->cur_scan.at(i) = Eigen::Tensor<float, 2>(4, this->MAX_POINTS);
        this->signals.at(i) = Eigen::Tensor<float, 2>(this->N_SIGNALS, this->MAX_POINTS);
    }

    this->indices.resize(this->N_FEATURES);
    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
        this->indices.at(i).resize(this->param.n_ring);
    }

    this->range_sensor = std::make_shared<RangeSensor>(param.sensor_params);

    this->undis_features.resize(this->N_FEATURES);

    if (this->param.num_trajectory_states < 2) {
        throw std::out_of_range("Number of parameter states must be at least 2");
    }
    if (this->param.n_window < 2) {
        throw std::out_of_range("Window size must be at least 2");
    }

    double step_size = 0.1 / (double) (this->param.num_trajectory_states - 1);

    if (params.output_trajectory) {
        long timestamp = std::chrono::system_clock::now().time_since_epoch().count();
        this->file.open(std::to_string(timestamp) + "laser_odom_traj.txt");
    }
}

void LaserOdom::updateParams(const LaserOdomParams new_params) {
    this->param = new_params;
}

LaserOdomParams LaserOdom::getParams() {
    return this->param;
}

LaserOdom::~LaserOdom() {
    if (this->param.output_trajectory) {
        this->file.close();
    }
    if (this->output_thread) {
        this->continue_output = false;
        this->output_condition.notify_one();
        this->output_thread->join();
    }

    delete this->CSVFormat;
}

void LaserOdom::registerOutputFunction(std::function<void()> output_function) {
    this->f_output = std::move(output_function);
    this->output_thread = std::make_unique<std::thread>(&LaserOdom::spinOutput, this);
}

void LaserOdom::spinOutput() {
    std::unique_lock<std::mutex> lk(this->output_mutex);
    while (this->continue_output) {
        while (!this->fresh_output) {  // wait can have spurious wakeups
            this->output_condition.wait(lk);
            if (!this->continue_output) {
                break;
            }
        }
        this->f_output();
        this->fresh_output = false;
    }
}

/**
 * This corrects for motion distortion of the current scan. Transforming it to the end.
 */
void LaserOdom::undistort() {
    Eigen::Tensor<float, 2> output;
    for (auto &line : this->cur_scan) {
    }
}

void LaserOdom::copyTrajectory() {
    for (uint32_t i = 0; i < this->param.num_trajectory_states; i++) {
        this->prev_trajectory.at(i).pose = this->cur_trajectory.at(i).pose;
        this->prev_trajectory.at(i).vel = this->cur_trajectory.at(i).vel;
    }
}

void LaserOdom::updateFeatureCandidates() {
    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
        std::swap(this->cur_feature_candidates.at(i), this->prev_feature_candidates.at(i));
        auto &feat = this->cur_feature_candidates.at(i);
        long featcnt = 0;
        for (const auto &elem : this->indices.at(i)) {
            featcnt += elem.dimension(0);
        }
        feat.resize(4, featcnt);
        long offset = 0;
        for (uint32_t j = 0; j < this->indices.at(i).size(); ++j) {
#pragma omp parallel for
            for (uint32_t k = 0; k < this->indices.at(i).at(j).dimension(0); ++k) {
                const int &idx = this->indices.at(i).at(j)(k);
                feat.slice(ar2({0, offset}), ar2({4, 1})) = this->cur_scan.at(j).slice(ar2({0, idx}), ar2({4, 1}));
            }
            offset += this->indices.at(i).at(j).dimension(0);
        }
    }
}

void LaserOdom::addPoints(const std::vector<PointXYZIR> &pts, const int tick, TimeType stamp) {
    if (tick - this->prv_tick < -200) {  // current scan has ended
        this->feature_extractor.getFeatures(this->cur_scan, this->signals, this->counters, this->indices);
        this->updateFeatureCandidates();
        if (this->initialized) {
            this->match();

            if (this->param.output_trajectory) {
                this->file << this->cur_trajectory.back().pose.storage.format(*(this->CSVFormat)) << std::endl;
            }
            if (this->output_thread) {
                {
                    std::unique_lock<std::mutex> lk(this->output_mutex);
                    if (this->fresh_output) {
                        // data from last time hasn't been consumed yet
                        LOG_ERROR("Overwriting previous output");
                    }
                    this->undistort_state.stamp = stamp;
                    this->undistort_state.pose = this->cur_trajectory.back().pose;
                    this->undistort_state.vel = this->cur_trajectory.back().vel;

                    this->undistort();
                    this->fresh_output = true;
                }
                this->output_condition.notify_one();
            }
        }
        this->rollover(stamp);
        std::fill(this->counters.begin(), this->counters.end(), 0);
    }

    for (PointXYZIR pt : pts) {
        if (counters.at(pt.ring) >= static_cast<int>(this->MAX_POINTS)) {
            throw std::out_of_range("Rebuild with higher max points");
        }
        this->cur_scan.at(pt.ring)(0, counters.at(pt.ring)) = pt.x;
        this->cur_scan.at(pt.ring)(1, counters.at(pt.ring)) = pt.y;
        this->cur_scan.at(pt.ring)(2, counters.at(pt.ring)) = pt.z;

        auto diff = stamp - this->scan_stamps_chrono.back();
        float secon = std::chrono::duration<float, std::ratio<1>>(diff).count();

        this->cur_scan.at(pt.ring)(3, counters.at(pt.ring)) = secon;

        this->signals.at(pt.ring)(0, counters.at(pt.ring)) = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        this->signals.at(pt.ring)(1, counters.at(pt.ring)) = pt.intensity;

        this->counters.at(pt.ring)++;
    }

    this->prv_tick = tick;
}

void LaserOdom::rollover(TimeType stamp) {
    // If there are n_scans worth of data, rotate
    if (this->scan_stamps_chrono.size() == this->param.n_window) {
        // Perform a left rotation
        std::rotate(this->feat_pts.begin(), this->feat_pts.begin() + 1, this->feat_pts.end());
        std::rotate(this->feat_pts_T.begin(), this->feat_pts_T.begin() + 1, this->feat_pts_T.end());
        std::rotate(this->scan_stamps_chrono.begin(), this->scan_stamps_chrono.begin() + 1, this->scan_stamps_chrono.end());
        this->scan_stamps_chrono.back() = stamp;
    } else {
        // grow storage
        this->scan_stamps_chrono.emplace_back(stamp);
        this->scan_stampsf.resize(this->scan_stamps_chrono.size());
        VecE<Eigen::Tensor<float, 2>> vec(this->N_FEATURES);
        this->feat_pts.emplace_back(std::move(vec));
        VecE<Eigen::Tensor<float, 2>> vec2(this->N_FEATURES);
        this->feat_pts_T.emplace_back(std::move(vec2));
    }
    for (uint32_t i = 0; i < this->scan_stamps_chrono.size(); i++) {
        auto diff = this->scan_stamps_chrono.at(i) - this->scan_stamps_chrono.front();
        this->scan_stampsf.at(i) = std::chrono::duration<float, std::ratio<1>>(diff).count();
    }

    for (unlong i = 0; i < this->param.n_ring; i++) {
        this->counters.at(i) = 0;
    }
    if (!this->initialized) {
        size_t feature_count = 0;
        for (uint32_t i = 0; i < this->prev_feature_candidates.size(); i++) {
            feature_count += this->prev_feature_candidates.at(i).dimension(1);
        }

        if (feature_count >= (size_t) (this->param.min_features)) {
            this->initialized = true;
        }
    }
    this->prior_twist = this->cur_trajectory.back().vel;
    this->cur_trajectory.back().pose.transformInverse(this->inv_prior_pose);

    this->cur_trajectory.front().pose = this->cur_trajectory.back().pose;

    for (uint32_t i = 1; i < this->param.num_trajectory_states; i++) {
        this->cur_trajectory.at(i).pose = this->cur_trajectory.at(i - 1).pose;
        this->cur_trajectory.at(i).pose.manifoldPlus(
          (this->param.scan_period / (this->param.num_trajectory_states - 1)) * this->cur_trajectory.back().vel);
    }
    // Now previous trajectory will hold the "motion generated" trajectory
    this->copyTrajectory();
}

bool LaserOdom::runOptimization(ceres::Problem &problem) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = this->param.max_inner_iters;
    options.max_num_consecutive_invalid_steps = 30;
    options.function_tolerance = 1e-8;
    options.parameter_tolerance = 1e-7;
    options.logging_type = ceres::LoggingType::SILENT;

    ceres::Covariance::Options covar_options;
    covar_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
    covar_options.algorithm_type = ceres::CovarianceAlgorithmType::SPARSE_QR;

    if (this->param.solver_threads < 1) {
        options.num_threads = std::thread::hardware_concurrency();
        options.num_linear_solver_threads = std::thread::hardware_concurrency();
        covar_options.num_threads = std::thread::hardware_concurrency();
    } else {
        options.num_threads = this->param.solver_threads;
        options.num_linear_solver_threads = this->param.solver_threads;
        covar_options.num_threads = this->param.solver_threads;
    }

    if (problem.NumResidualBlocks() < this->param.min_residuals) {
        LOG_ERROR("Less than expected residuals, resetting");
        LOG_ERROR("%d residuals, threshold is %d", problem.NumResidualBlocks(), this->param.min_residuals);
        this->resetTrajectory();
        this->initialized = false;
        return false;
    } else if (!this->param.only_extract_features) {
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if (this->param.plot_stuff) {
            LOG_INFO("%s", summary.FullReport().c_str());
        }
        //                ceres::Covariance covariance(covar_options);
        //                if (!covariance.Compute(this->param_blocks, &problem)) {
        //                    LOG_ERROR("covariance did not compute");
        //                }
        //                covariance.GetCovarianceMatrixInTangentSpace(this->param_blocks, this->covar.data());
    }
    return true;
}

void LaserOdom::extendFeatureTracks(const Eigen::MatrixXi &idx, const Eigen::MatrixXf &dist, uint32_t feat_id) {
    uint32_t knn = 3;
    if (this->feature_residuals.at(feat_id) == PointToLine) {
        knn = 2;
    }
    /// Each column represents a query (feature track)
    Eigen::Tensor<float, 2> new_feat_points(4, 2000);
    auto offset = this->feat_pts.back().at(feat_id).dimension(1);
    long new_feat_cnt = 0;
    for (uint32_t j = 0; j < idx.cols(); ++j) {
        std::vector<uint32_t> matches;
        double min_elev = 0.0;
        double max_elev = 0.0;
        bool wide_spread = false;
        /// Each row is a nearest neighbour
        for (uint32_t i = 0; i < idx.rows(); ++i) {
            if (dist(i, j) > this->param.max_correspondence_dist) {
                continue;
            }
            Vec3f pt = this->cur_feat_map.at(feat_id).block<3, 1>(0, idx(i, j));
            if (matches.empty()) {
                matches.emplace_back(idx(i, j));
                min_elev = std::atan2(pt(2), std::sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
                max_elev = min_elev;
                continue;
            }
            if (wide_spread || matches.size() + 1 != knn) {
                if (!wide_spread) {
                    double new_elev = std::atan2(pt(2), std::sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
                    if (new_elev > max_elev)
                        max_elev = new_elev;
                    else if (new_elev < min_elev)
                        min_elev = new_elev;
                    if (max_elev - min_elev > this->param.azimuth_tol)
                        wide_spread = true;
                }
                matches.emplace_back(idx(i, j));
            }
            if (matches.size() == knn)
                break;
        }
        /// add to track if error against current landmark is low
        if (matches.size() == knn) {
            for (const auto &elem : matches) {
                if (this->cur_feat_idx.at(feat_id).at(elem) != -1) {
                    continue;
                }
                const auto &geometry = this->feature_tracks.at(feat_id).at(j).geometry;
                const auto &pt = this->cur_feat_map.at(feat_id).block<3, 1>(0, elem);
                Vec3 diff = (pt.cast<double>() - geometry.block<3, 1>(3, 0));
                Vec1 error;
                if (this->feature_residuals.at(feat_id) == PointToLine) {
                    Vec3 err = (diff - geometry.block<3, 1>(0, 0) * (diff.transpose() * geometry.block<3, 1>(0, 0)));
                    error = err.transpose() * err;
                } else {
                    error = (diff.transpose() * geometry.block<3, 1>(0, 0));
                    error = error.transpose() * error;
                }
                // copy candidate point into the feature point set (both original and transformed) if it is not already
                // used in another residual
                if (error(0) < sqrt(this->param.max_residual_val)) {
                    this->cur_feat_idx.at(feat_id).at(elem) = j;

                    Eigen::array<int, 2> offsets_new = {0, static_cast<int>(new_feat_cnt)};
                    Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(elem)};
                    Eigen::array<int, 2> extents = {4, 1};

                    new_feat_points.slice(offsets_new, extents) =
                      this->cur_feature_candidates.at(feat_id).slice(offsets_candidate, extents);

                    this->feature_tracks.at(feat_id).at(j).mapping.emplace_back(offset + new_feat_cnt,
                                                                                this->param.n_window - 1);
                    std::vector<uint32_t> states(4);
                    auto bnd = std::upper_bound(
                      this->trajectory_stamps.begin(), this->trajectory_stamps.end(), new_feat_points(3, new_feat_cnt));
                    auto traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);
                    states.at(0) = traj_idx * 2;
                    states.at(1) = states.at(0) + 1;
                    states.at(2) = states.at(1) + 1;
                    states.at(3) = states.at(2) + 1;
                    this->feature_tracks.at(feat_id).at(j).state_ids.emplace_back(states);
                    this->feature_tracks.at(feat_id).at(j).length += 1;

                    ++new_feat_cnt;
                }
            }
        }
    }

    Eigen::array<long int, 2> offsets = {0, 0};
    Eigen::array<long int, 2> extents = {4, static_cast<long int>(new_feat_cnt)};
    new_feat_points = new_feat_points.slice(offsets, extents);
    this->feat_pts.back().at(feat_id) = this->feat_pts.back().at(feat_id).concatenate(new_feat_points, 1);
}

// todo reduce duplicated code
void LaserOdom::createNewFeatureTracks(const Eigen::MatrixXi &idx, const Eigen::MatrixXf &dist, uint32_t feat_id) {
    uint32_t knn = 3;
    if (this->feature_residuals.at(feat_id) == PointToLine) {
        knn = 2;
    }
    /// Each column represents a query (feature track)
    Eigen::Tensor<float, 2> new_feat_points(4, 2000), prev_new_feat_points(4, 2000);
    long new_feat_cnt = 0, prev_new_feat_cnt = 0;
    auto prev_offset = this->feat_pts.at(this->param.n_window - 2).at(feat_id).dimension(1);
    auto cur_offset = this->feat_pts.at(this->param.n_window - 1).at(feat_id).dimension(1);
    for (uint32_t j = 0; j < idx.cols(); ++j) {
        if (this->cur_feat_idx.at(feat_id).at(j) != -1) {
            continue;
        }

        std::vector<uint32_t> matches;
        double min_elev = 0.0;
        double max_elev = 0.0;
        bool wide_spread = false;
        /// Each row is a nearest neighbour
        for (uint32_t i = 0; i < idx.rows(); ++i) {
            if (this->prev_feat_idx.at(feat_id).at(idx(i, j)) != -1) {
                continue;
            }
            if (dist(i, j) > this->param.max_correspondence_dist) {
                continue;
            }
            Vec3f pt = this->cur_feat_map.at(feat_id).block<3, 1>(0, j);
            if (matches.empty()) {
                matches.emplace_back(idx(i, j));
                min_elev = std::atan2(pt(2), std::sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
                max_elev = min_elev;
                continue;
            }
            if (wide_spread || matches.size() + 1 != knn) {
                if (!wide_spread) {
                    double new_elev = std::atan2(pt(2), std::sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
                    if (new_elev > max_elev)
                        max_elev = new_elev;
                    else if (new_elev < min_elev)
                        min_elev = new_elev;
                    if (max_elev - min_elev > this->param.azimuth_tol)
                        wide_spread = true;
                }
                matches.emplace_back(idx(i, j));
            }
            if (matches.size() == knn)
                break;
        }
        /// create feature track if error is low, and points are not part of another track
        if (matches.size() == knn) {
            // Depending on the type of geometry, need to initialize feature track with reasonable estimate
            Vec6 geometry = Vec6::Zero();

            if (this->feature_residuals.at(feat_id) == PointToLine) {
                geometry.block<3, 1>(3, 0) =
                  (this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(0)).cast<double>() +
                   this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(1)).cast<double>()) /
                  2.0;

                geometry.block<3, 1>(0, 0) =
                  this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(0)).cast<double>() -
                  this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(1)).cast<double>();
            } else {
                geometry.block<3, 1>(3, 0) =
                  (this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(0)).cast<double>() +
                   this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(1)).cast<double>() +
                   this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(2)).cast<double>()) /
                  3.0;

                Vec3f a = this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(0)) -
                          this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(1));
                Vec3f b = this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(0)) -
                          this->prev_feat_map.at(feat_id).block<3, 1>(0, matches.at(2));
                geometry.block<3, 1>(0, 0) = a.cross(b).cast<double>();
            }
            geometry.block<3, 1>(3, 0).normalize();

            Vec3 diff = (this->cur_feat_map.at(feat_id).block<3, 1>(0, j).cast<double>() - geometry.block<3, 1>(3, 0));
            Vec1 error;
            if (this->feature_residuals.at(feat_id) == PointToLine) {
                Vec3 err = (diff - geometry.block<3, 1>(0, 0) * (diff.transpose() * geometry.block<3, 1>(0, 0)));
                error = err.transpose() * err;
            } else {
                error = (diff.transpose() * geometry.block<3, 1>(0, 0));
                error = error.transpose() * error;
            }

            // create new residual, may need to increase tolerance a bit to find new features when initializing
            if (error(0) < sqrt(this->param.max_residual_val)) {
                // create feature track and add queried point
                this->feature_tracks.at(feat_id).emplace_back(FeatureTrack());
                auto &track = this->feature_tracks.at(feat_id).back();
                track.geometry = geometry;
                track.length = 1;
                track.mapping.emplace_back(cur_offset + new_feat_cnt, this->param.n_window - 1);

                Eigen::array<int, 2> offsets_new = {0, static_cast<int>(new_feat_cnt)};
                Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(j)};
                Eigen::array<int, 2> extents = {4, 1};

                new_feat_points.slice(offsets_new, extents) =
                  this->cur_feature_candidates.at(feat_id).slice(offsets_candidate, extents);
                this->cur_feat_idx.at(feat_id).at(j) = static_cast<int>(this->feature_tracks.size()) - 1;

                ++new_feat_cnt;

                // now add points from previous scan
                for (auto elem : matches) {
                    track.mapping.emplace_back(prev_offset + prev_new_feat_cnt, this->param.n_window - 2);

                    offsets_new.at(1) = static_cast<int>(prev_new_feat_cnt);
                    offsets_candidate.at(1) = static_cast<int>(elem);

                    prev_new_feat_points.slice(offsets_new, extents) =
                      this->prev_feature_candidates.at(feat_id).slice(offsets_candidate, extents);
                    this->prev_feat_idx.at(feat_id).at(elem) = static_cast<int>(this->feature_tracks.size()) - 1;

                    ++prev_new_feat_cnt;
                }
            }
        }
    }

    Eigen::array<long int, 2> offsets = {0, 0};
    Eigen::array<long int, 2> extents = {4, static_cast<long int>(new_feat_cnt)};
    new_feat_points = new_feat_points.slice(offsets, extents);
    this->feat_pts.back().at(feat_id) = this->feat_pts.back().at(feat_id).concatenate(new_feat_points, 1);

    extents[1] = static_cast<long int>(prev_new_feat_cnt);
    prev_new_feat_points = prev_new_feat_points.slice(offsets, extents);
    this->feat_pts.at(this->param.n_window - 2).at(feat_id) =
      this->feat_pts.at(this->param.n_window - 2)
        .at(feat_id)
        .concatenate(prev_new_feat_points, 1);
}

void LaserOdom::mergeFeatureTracks(uint32_t) {}

bool LaserOdom::match() {
    T_TYPE last_transform;
    auto &ref = this->cur_trajectory.back().pose;

    ceres::Problem::Options options;
    options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(options);

    for (int op = 0; op < this->param.opt_iters; op++) {
        if (op > 0) {
            last_transform = ref;
        }

        this->transformer.update(this->cur_trajectory, this->scan_stampsf);

        for (uint32_t j = 0; j < this->N_FEATURES; j++) {
            /// 1. Transform all features to the start of the window
            for (uint32_t i = 0; i < this->feat_pts.size(); i++) {
                auto &feat = this->feat_pts.at(i).at(j);
                auto &featT = this->feat_pts_T.at(i).at(j);
                this->transformer.transformToStart(feat, featT, i);
            }
            /// 1.5 Transform all candidate points to the start of the window
            this->transformer.transformToStart(this->cur_feature_candidates.at(j), this->cur_feature_candidatesT.at(j), this->scan_stamps_chrono.size() - 1);
            this->transformer.transformToStart(this->prev_feature_candidates.at(j), this->prev_feature_candidatesT.at(j), this->scan_stamps_chrono.size() - 2);
            auto &cfeat = this->cur_feature_candidatesT.at(j);
            this->cur_feat_map.at(j) =
              Eigen::Map<Eigen::MatrixXf>(cfeat.data(), cfeat.dimension(0), cfeat.dimension(1));

            auto &pfeat = this->prev_feature_candidatesT.at(j);
            this->prev_feat_map.at(j) =
              Eigen::Map<Eigen::MatrixXf>(pfeat.data(), pfeat.dimension(0), pfeat.dimension(1));

            /// 2. Update average point position from each feature tracks
            this->ave_pts.at(j).resize(3, this->feature_tracks.at(j).size());
#pragma omp parallel for
            for (uint32_t t_idx = 0; t_idx < this->feature_tracks.at(j).size(); t_idx++) {
                const auto &track = this->feature_tracks.at(j).at(t_idx);
                this->ave_pts.at(j).block<3, 1>(0, t_idx) = track.geometry.block<3, 1>(3, 0).cast<float>();
            }
            /// 3. Build kd trees on previous two scans, and on average track locations
            delete this->cur_kd_idx.at(j);
            delete this->curm1_kd_idx.at(j);
            delete this->ave_kd_idx.at(j);
            this->cur_kd_idx.at(j) = Nabo::NNSearchF::createKDTreeLinearHeap(this->cur_feat_map.at(j));
            this->curm1_kd_idx.at(j) = Nabo::NNSearchF::createKDTreeLinearHeap(this->prev_feat_map.at(j));
            this->ave_kd_idx.at(j) = Nabo::NNSearchF::createKDTreeLinearHeap(this->ave_pts.at(j));

            /// 4. Find correspondences for existing feature tracks in current scan
            Eigen::MatrixXi nn_idx;
            Eigen::MatrixXf nn_dist;
            this->cur_kd_idx.at(j)->knn(this->ave_pts.at(j), nn_idx, nn_dist, 5, 0.1, Nabo::NNSearchF::SORT_RESULTS);
            if (op == 0) {
                this->cur_feat_idx.at(j).resize(static_cast<unsigned long>(this->cur_feat_map.at(j).size()));
                std::fill(this->cur_feat_idx.at(j).begin(), this->cur_feat_idx.at(j).end(), -1);
            }
            this->extendFeatureTracks(nn_idx, nn_dist, j);

            /// 5. Create new feature tracks between new and old scan
            this->curm1_kd_idx.at(j)->knn(
              this->cur_feat_map.at(j), nn_idx, nn_dist, 5, 0.1, Nabo::NNSearchF::SORT_RESULTS);
            this->createNewFeatureTracks(nn_idx, nn_dist, j);

            /// 6. Merge feature tracks (optional)
            this->mergeFeatureTracks(j);

            /// 7. Build Feature Residuals
            this->buildResiduals(problem);

            if (!this->runOptimization(problem))
                return false;
        }
    }
    return true;
}

void LaserOdom::buildResiduals(ceres::Problem &problem) {
    this->costs.clear();
    this->local_params.clear();
    this->loss_functions.clear();
    for (uint32_t f_idx = 0; f_idx < this->N_FEATURES; ++f_idx) {
        for (auto &track : this->feature_tracks.at(f_idx)) {
            // plane_cost
            if (f_idx == 2) {
                this->local_params.emplace_back(new PlaneParameterization());
            } else {
                this->local_params.emplace_back(new LineParameterization());
            }
            problem.SetParameterization(track.geometry.data(), this->local_params.back().get());
            for (uint32_t p_idx = 0; p_idx < track.mapping.size(); ++p_idx) {
                this->loss_functions.emplace_back(new BisquareLoss(this->param.robust_param));
                if (f_idx == 2) {
                    this->costs.emplace_back(
                      new ImplicitPlaneResidual<12, 6, 12, 6>(p_idx, f_idx, &track, &(this->feat_T_map)));
                } else {
                    this->costs.emplace_back(
                      new ImplicitLineResidual<12, 6, 12, 6>(p_idx, f_idx, &track, &(this->feat_T_map)));
                }
                uint32_t start_offset = track.state_ids.at(p_idx).at(0) / 2;
                problem.AddResidualBlock(this->costs.back().get(),
                                         this->loss_functions.back().get(),
                                         track.geometry.data(),
                                         this->cur_trajectory.at(start_offset).pose.storage.data(),
                                         this->cur_trajectory.at(start_offset).vel.data(),
                                         this->cur_trajectory.at(start_offset + 1).pose.storage.data(),
                                         this->cur_trajectory.at(start_offset + 1).vel.data());
            }
        }
    }
    for (auto &state : this->cur_trajectory) {
        this->local_params.emplace_back(new NullSE3Parameterization);
        problem.SetParameterization(state.pose.storage.data(), this->local_params.back().get());
    }
}

void LaserOdom::resetTrajectory() {
    for (auto tra : this->cur_trajectory) {
        tra.pose.setIdentity();
        tra.vel.setZero();
    }
    this->prior_twist.setZero();
}

}  // namespace wave
