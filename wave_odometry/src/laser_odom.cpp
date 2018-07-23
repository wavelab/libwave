#include "wave/odometry/laser_odom.hpp"

namespace wave {

LaserOdom::LaserOdom(const LaserOdomParams params, const FeatureExtractorParams feat_params,
                     const TransformerParams transformer_params)
    : param(params), feature_extractor(feat_params, params.n_ring), transformer(transformer_params) {
    this->CSVFormat = new Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ");

    this->cv_model = std::make_shared<wave_kinematics::ConstantVelocityPrior>(0, 0, nullptr, this->param.Qc, this->param.inv_Qc);

    auto n_ring = static_cast<size_t>(param.n_ring);
    this->feature_extractor.setParams(feat_params, n_ring);
    this->counters.resize(n_ring);
    std::fill(this->counters.begin(), this->counters.end(), 0);

    //todo don't do this
    this->cur_feature_candidates.resize(this->N_FEATURES);
    this->cur_feature_candidatesT.resize(this->N_FEATURES);
    this->prev_feature_candidates.resize(this->N_FEATURES);
    this->prev_feature_candidatesT.resize(this->N_FEATURES);
    this->cur_feat_map.resize(this->N_FEATURES);
    this->prev_feat_map.resize(this->N_FEATURES);
    this->ave_pts.resize(this->N_FEATURES);
    this->feature_tracks.resize(this->N_FEATURES);
    this->cur_kd_idx.resize(this->N_FEATURES);
    this->curm1_kd_idx.resize(this->N_FEATURES);
    this->ave_kd_idx.resize(this->N_FEATURES);
    this->cur_feat_idx.resize(this->N_FEATURES);
    this->prev_feat_idx.resize(this->N_FEATURES);
    this->ptT_jacobians.resize(this->N_FEATURES);

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
        std::swap(this->cur_feat_idx.at(i), this->prev_feat_idx.at(i));
        auto &feat = this->cur_feature_candidates.at(i);
        long featcnt = 0;
        for (const auto &elem : this->indices.at(i)) {
            featcnt += elem.dimension(0);
        }
        feat.resize(4, featcnt);
        this->cur_feat_idx.at(i).resize(featcnt);
        std::fill(this->cur_feat_idx.at(i).begin(), this->cur_feat_idx.at(i).end(), -1);
        long offset = 0;
        for (uint32_t j = 0; j < this->indices.at(i).size(); ++j) {
//#pragma omp parallel for
            for (uint32_t k = 0; k < this->indices.at(i).at(j).dimension(0); ++k) {
                const int &idx = this->indices.at(i).at(j)(k);
                feat.slice(ar2({0, offset}), ar2({4, 1})) = this->cur_scan.at(j).slice(ar2({0, idx}), ar2({4, 1}));
                ++offset;
            }
        }
    }
}

void LaserOdom::addPoints(const std::vector<PointXYZIR> &pts, const int tick, TimeType stamp) {
    if (tick - this->prv_tick < -200) {  // current scan has ended
        this->feature_extractor.getFeatures(this->cur_scan, this->signals, this->counters, this->indices);
        this->updateFeatureCandidates();
        if (this->initialized) {
            this->match(stamp);

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
        std::rotate(
          this->scan_stamps_chrono.begin(), this->scan_stamps_chrono.begin() + 1, this->scan_stamps_chrono.end());

        std::rotate(this->cur_trajectory.begin(),
                    this->cur_trajectory.begin() + this->param.num_trajectory_states - 1,
                    this->cur_trajectory.end());

        // adjust timestamps and trajectory states to start of new window
        for (uint32_t i = 0; i < this->scan_stampsf.size() - 1; ++i) {
            for (uint32_t j = 0; j < this->param.num_trajectory_states - 1; ++j) {
                this->cur_trajectory.at(i * this->param.num_trajectory_states + j).pose =
                  this->cur_trajectory.front().pose.transformInverse() *
                  this->cur_trajectory.at(i * this->param.num_trajectory_states + j).pose;
            }
        }
        this->scan_stamps_chrono.back() = stamp;
        auto index = (this->param.num_trajectory_states - 1) * (this->scan_stampsf.size() - 1);
        this->cur_trajectory.at(index).pose =
                this->cur_trajectory.front().pose.transformInverse() *
                this->cur_trajectory.at(index).pose;
    } else {
        // grow storage
        this->scan_stamps_chrono.emplace_back(stamp);
        this->scan_stampsf.resize(this->scan_stamps_chrono.size());
        VecE<Eigen::Tensor<float, 2>> vec(this->N_FEATURES);
        VecE<Eigen::Tensor<float, 2>> vec2(this->N_FEATURES);

        for (auto &elem : vec) {
            elem.resize(4, 0);
        }

        this->feat_pts.emplace_back(std::move(vec));
        this->feat_pts_T.emplace_back(std::move(vec2));

        this->feat_T_map.resize(this->feat_T_map.size() + 1);
        this->feat_T_map.back().resize(this->N_FEATURES);

        for (auto &elem : this->ptT_jacobians) {
            elem.emplace_back(VecE<Eigen::Tensor<double, 3>>(4));
        }

        if (this->cur_trajectory.empty()) {
            this->cur_trajectory.resize(this->param.num_trajectory_states);
            this->trajectory_stamps.resize(this->param.num_trajectory_states);
        } else {
            this->cur_trajectory.resize(this->cur_trajectory.size() + this->param.num_trajectory_states - 1);
            this->trajectory_stamps.resize(this->trajectory_stamps.size() + this->param.num_trajectory_states - 1);
        }
        this->prev_trajectory.resize(this->cur_trajectory.size());
    }

    uint32_t mult = this->param.num_trajectory_states - 1;
    for (uint32_t i = 0; i < this->scan_stamps_chrono.size(); i++) {
        auto diff = this->scan_stamps_chrono.at(i) - this->scan_stamps_chrono.front();
        this->scan_stampsf.at(i) = std::chrono::duration<float, std::ratio<1>>(diff).count();
        this->trajectory_stamps.at(i * mult) = this->scan_stampsf.at(i);
        if (i != 0) {
            float step = (this->scan_stampsf.at(i) - this->scan_stampsf.at(i - 1)) / mult;
            for (uint32_t j = 1; j < mult; ++j) {
                this->trajectory_stamps.at((i - 1) * mult + j) = this->trajectory_stamps.at((i - 1) * mult) + j * step;
            }
        }
    }

    std::fill(this->counters.begin(), this->counters.end(), 0);

    if (!this->initialized) {
        size_t feature_count = 0;
        for (const auto &cand : this->prev_feature_candidates) {
            feature_count += cand.dimension(1);
        }

        if (feature_count >= (size_t)(this->param.min_features)) {
            this->initialized = true;
        }
    }
}

bool LaserOdom::runOptimization(ceres::Problem &problem) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = this->param.max_inner_iters;
    options.max_num_consecutive_invalid_steps = 30;
    options.function_tolerance = 1e-8;
    options.parameter_tolerance = 1e-7;
    options.logging_type = ceres::LoggingType::SILENT;
    options.use_nonmonotonic_steps = true;

    auto callback = new OdometryCallback(&(this->feat_pts), &(this->feat_pts_T), &(this->cur_trajectory), &(this->ptT_jacobians), &(this->trajectory_stamps), &(this->scan_stampsf), &(this->transformer));
    options.evaluation_callback = callback;

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
    uint32_t knn = 2;
    auto residualType = PointToLine;
    //todo don't do this
    if (feat_id == 2) {
        knn = 3;
        residualType = PointToPlane;
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
            Vec3f pt = this->cur_feat_map.at(feat_id)->block<3, 1>(0, idx(i, j));
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
                const auto &pt = this->cur_feat_map.at(feat_id)->block<3, 1>(0, elem);
                Vec3 diff = (pt.cast<double>() - geometry.block<3, 1>(3, 0));
                Vec1 error;
                if (residualType == PointToLine) {
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
                                                                                this->scan_stampsf.size() - 1);
                    auto bnd = std::upper_bound(
                      this->trajectory_stamps.begin(), this->trajectory_stamps.end(), new_feat_points(3, new_feat_cnt));
                    auto traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);

                    this->feature_tracks.at(feat_id).at(j).state_ids.emplace_back(traj_idx);
                    this->feature_tracks.at(feat_id).at(j).length += 1;

                    ++new_feat_cnt;
                }
            }
        }
    }

    auto &cur_feat_points = this->feat_pts.back().at(feat_id);
    Eigen::TensorMap<Eigen::Tensor<float, 2>> reduced_new_feat_points(new_feat_points.data(), 4, new_feat_cnt);
    if (cur_feat_points.dimension(1) == 0) {
        cur_feat_points = reduced_new_feat_points;
    } else if (reduced_new_feat_points.dimension(1) != 0) {
        Eigen::Tensor<float, 2> stupid_eigen = cur_feat_points.concatenate(reduced_new_feat_points, 1);
        cur_feat_points = stupid_eigen;
    }
}

// queries are newest scan, searching next newer scan
// todo reduce duplicated code
void LaserOdom::createNewFeatureTracks(const Eigen::MatrixXi &idx, const Eigen::MatrixXf &dist, uint32_t feat_id) {
    uint32_t knn = 2;
    auto residualType = PointToLine;
    //todo don't do this
    if (feat_id == 2) {
        knn = 3;
        residualType = PointToPlane;
    }
    /// Each column represents a query (feature track)
    Eigen::Tensor<float, 2> new_feat_points(4, 2000), prev_new_feat_points(4, 2000);
    long new_feat_cnt = 0, prev_new_feat_cnt = 0;
    auto &prev_feat_points = this->feat_pts.at(this->feat_pts.size() - 2).at(feat_id);
    auto &cur_feat_points = this->feat_pts.at(this->feat_pts.size() - 1).at(feat_id);
    auto prev_offset = prev_feat_points.dimension(1);
    auto cur_offset = cur_feat_points.dimension(1);
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
            Vec3f pt = this->prev_feat_map.at(feat_id)->block<3, 1>(0, idx(i, j));
            if (matches.empty()) {
                matches.emplace_back(idx(i, j));
                min_elev = std::atan2(pt(2), std::sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
                max_elev = min_elev;
                continue;
            }
            if (wide_spread || matches.size() != knn) {
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

            if (residualType == PointToLine) {
                geometry.block<3, 1>(3, 0) =
                  (this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)).cast<double>() +
                   this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(1)).cast<double>()) /
                  2.0;

                geometry.block<3, 1>(0, 0) =
                  this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)).cast<double>() -
                  this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(1)).cast<double>();
            } else {
                geometry.block<3, 1>(3, 0) =
                  (this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)).cast<double>() +
                   this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(1)).cast<double>() +
                   this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(2)).cast<double>()) /
                  3.0;

                Vec3f a = this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)) -
                          this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(1));
                Vec3f b = this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)) -
                          this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(2));
                geometry.block<3, 1>(0, 0) = a.cross(b).cast<double>();
            }
            geometry.block<3, 1>(0, 0).normalize();
            if (geometry(2) < 0) {
                geometry.block<3, 1>(0,0) = -geometry.block<3, 1>(0,0);
            }

            Vec3 diff = (this->cur_feat_map.at(feat_id)->block<3, 1>(0, j).cast<double>() - geometry.block<3, 1>(3, 0));
            Vec1 error;
            if (residualType == PointToLine) {
                Vec3 err = (diff - geometry.block<3, 1>(0, 0) * (diff.transpose() * geometry.block<3, 1>(0, 0)));
                error = err.transpose() * err;
            } else {
                error = (diff.transpose() * geometry.block<3, 1>(0, 0));
                error = error.transpose() * error;
            }

            // create new residual, may need to increase tolerance a bit to find new features when initializing
            if (error(0) < this->param.max_residual_val * this->param.max_residual_val) {
                // create feature track and add queried point
                this->feature_tracks.at(feat_id).emplace_back(FeatureTrack());
                auto &track = this->feature_tracks.at(feat_id).back();
                track.geometry = geometry;
                track.length = 1;
                track.jacs = &(this->ptT_jacobians.at(feat_id));
                track.mapping.emplace_back(cur_offset + new_feat_cnt, this->scan_stampsf.size() - 1);

                float pt_time = this->cur_feature_candidates.at(feat_id)(3, j) + this->scan_stampsf.back();
                auto bnd = std::upper_bound(
                        this->trajectory_stamps.begin(), this->trajectory_stamps.end(), pt_time);
                auto traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);

                track.state_ids.emplace_back(traj_idx);

                Eigen::array<int, 2> offsets_new = {0, static_cast<int>(new_feat_cnt)};
                Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(j)};
                Eigen::array<int, 2> extents = {4, 1};

                new_feat_points.slice(offsets_new, extents) =
                  this->cur_feature_candidates.at(feat_id).slice(offsets_candidate, extents);
                this->cur_feat_idx.at(feat_id).at(j) = static_cast<int>(this->feature_tracks.size()) - 1;

                ++new_feat_cnt;

                // now add points from previous scan
                for (auto elem : matches) {
                    track.mapping.emplace_back(prev_offset + prev_new_feat_cnt, this->scan_stampsf.size() - 2);

                    pt_time = this->prev_feature_candidates.at(feat_id)(3, elem) + this->scan_stampsf.at(this->scan_stampsf.size() - 2);
                    bnd = std::upper_bound(
                            this->trajectory_stamps.begin(), this->trajectory_stamps.end(), pt_time);
                    traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);

                    track.state_ids.emplace_back(traj_idx);

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

    Eigen::TensorMap<Eigen::Tensor<float, 2>> reduced_new_feat_points(new_feat_points.data(), 4, new_feat_cnt);
    if (cur_feat_points.dimension(1) == 0) {
        cur_feat_points = reduced_new_feat_points;
    } else if (reduced_new_feat_points.dimension(1) != 0) {
        Eigen::Tensor<float, 2> stupid_eigen = cur_feat_points.concatenate(reduced_new_feat_points, 1);
        cur_feat_points = stupid_eigen;
    }

    Eigen::TensorMap<Eigen::Tensor<float, 2>> reduced_prev_new_feat_points(prev_new_feat_points.data(), 4, prev_new_feat_cnt);
    if (prev_feat_points.dimension(1) == 0) {
        prev_feat_points = reduced_prev_new_feat_points;
    } else if (reduced_prev_new_feat_points.dimension(1) != 0) {
        Eigen::Tensor<float, 2> stupid_eigen = prev_feat_points.concatenate(reduced_prev_new_feat_points, 1);
        prev_feat_points = stupid_eigen;
    }
}

void LaserOdom::mergeFeatureTracks(uint32_t) {}

void LaserOdom::prepTrajectory(const TimeType &stamp) {
    // scan stamps f holds the timestamps at the start of each scan
    // need to update the latter half of trajectory stamps based on the difference between
    // stamp and the last timestamp in stamps chrono.
    auto diff = stamp - this->scan_stamps_chrono.back();
    float scan_duration = std::chrono::duration<float, std::ratio<1>>(diff).count();

    // set up timestamps and trajectory for current scan
    float step = scan_duration / (this->param.num_trajectory_states - 1);

    unlong idx = this->trajectory_stamps.size() - this->param.num_trajectory_states + 1;
    for ( ; idx < this->trajectory_stamps.size(); ++idx) {
        this->trajectory_stamps.at(idx) = this->trajectory_stamps.at(idx - 1) + step;

        this->cur_trajectory.at(idx).pose = this->cur_trajectory.at(idx - 1).pose.manifoldPlus(
                step * this->cur_trajectory.at(idx - 1).vel);
        this->cur_trajectory.at(idx).vel = this->cur_trajectory.at(idx - 1).vel;
    }
    // as to limit floating point error
    this->trajectory_stamps.back() = std::chrono::duration<float, std::ratio<1>>(
            stamp - this->scan_stamps_chrono.front()).count();

    this->prior_twist = this->cur_trajectory.back().vel;

    // Now previous trajectory will hold the "motion generated" trajectory
    this->copyTrajectory();
}

bool LaserOdom::match(const TimeType &stamp) {
    const int knn = 5;
    this->prepTrajectory(stamp);

    T_TYPE last_transform;
    auto &ref = this->cur_trajectory.back().pose;

    for (int op = 0; op < this->param.opt_iters; op++) {
        if (op > 0) {
            last_transform = ref;
        }

        this->transformer.update(this->cur_trajectory, this->trajectory_stamps);

        for (uint32_t j = 0; j < this->N_FEATURES; j++) {
            /// 1 Transform all candidate points to the start of the window
            this->transformer.transformToStart(this->cur_feature_candidates.at(j),
                                               this->cur_feature_candidatesT.at(j),
                                               this->scan_stamps_chrono.size() - 1);
            this->transformer.transformToStart(this->prev_feature_candidates.at(j),
                                               this->prev_feature_candidatesT.at(j),
                                               this->scan_stamps_chrono.size() - 2);
            auto &cfeat = this->cur_feature_candidatesT.at(j);
            this->cur_feat_map.at(j) = std::make_shared<Eigen::Map<MatXf>>(cfeat.data(), cfeat.dimension(0), cfeat.dimension(1));

            auto &pfeat = this->prev_feature_candidatesT.at(j);
            this->prev_feat_map.at(j) = std::make_shared<Eigen::Map<MatXf>>(pfeat.data(), pfeat.dimension(0), pfeat.dimension(1));

            /// 2. Update average point position from each feature tracks
            this->ave_pts.at(j).resize(3, this->feature_tracks.at(j).size());
//#pragma omp parallel for
            for (uint32_t t_idx = 0; t_idx < this->feature_tracks.at(j).size(); t_idx++) {
                const auto &track = this->feature_tracks.at(j).at(t_idx);
                this->ave_pts.at(j).block<3, 1>(0, t_idx) = track.geometry.block<3, 1>(3, 0).cast<float>();
            }
            /// 3. Build kd trees on previous two scans, and on average track locations
            delete this->cur_kd_idx.at(j);
            delete this->curm1_kd_idx.at(j);
            delete this->ave_kd_idx.at(j);
            if (this->cur_feat_map.at(j)->cols() > 10) {
                this->cur_kd_idx.at(j) = Nabo::NNSearchF::createKDTreeLinearHeap(*(this->cur_feat_map.at(j)));
                Eigen::MatrixXi nn_idx;
                Eigen::MatrixXf nn_dist;
                if (!this->feature_tracks.at(j).empty()) {
                    this->ave_kd_idx.at(j) = Nabo::NNSearchF::createKDTreeLinearHeap(this->ave_pts.at(j));

                    /// 4. Find correspondences for existing feature tracks in current scan
                    nn_idx.resize(knn, this->ave_pts.at(j).cols());
                    nn_dist.resize(knn, this->ave_pts.at(j).cols());
                    this->cur_kd_idx.at(j)->knn(this->ave_pts.at(j), nn_idx, nn_dist, 5, 0.1, Nabo::NNSearchF::SORT_RESULTS);
                    if (op == 0) {
                        this->cur_feat_idx.at(j).resize(static_cast<unsigned long>(this->cur_feat_map.at(j)->size()));
                        std::fill(this->cur_feat_idx.at(j).begin(), this->cur_feat_idx.at(j).end(), -1);
                    }
                    this->extendFeatureTracks(nn_idx, nn_dist, j);
                }
                if (this->prev_feat_map.at(j)->cols() > 10) {
                    this->curm1_kd_idx.at(j) = Nabo::NNSearchF::createKDTreeLinearHeap(*(this->prev_feat_map.at(j)));
                    /// 5. Create new feature tracks between new and old scan
                    nn_idx.resize(knn, this->cur_feat_map.at(j)->cols());
                    nn_dist.resize(knn, this->cur_feat_map.at(j)->cols());
                    this->curm1_kd_idx.at(j)->knn(
                            *(this->cur_feat_map.at(j)), nn_idx, nn_dist, 5, 0.1, Nabo::NNSearchF::SORT_RESULTS);
                    this->createNewFeatureTracks(nn_idx, nn_dist, j);
                }
            }

            /// 6. Merge feature tracks (optional)
            this->mergeFeatureTracks(j);

            /// 6.5 Transform all features to the start of the window
            for (uint32_t i = 0; i < this->feat_pts.size(); i++) {
                auto &feat = this->feat_pts.at(i).at(j);
                auto &featT = this->feat_pts_T.at(i).at(j);
                this->transformer.transformToStart(feat, featT, i);
            }
        }
        /// 7. Build Feature Residuals. todo, reuse problem
        {
            ceres::Problem::Options options;
            options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            // live on the edge
            options.disable_all_safety_checks = true;
            ceres::Problem problem(options);
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
        for (uint32_t s_idx = 0; s_idx < this->feat_pts_T.size(); ++s_idx) {
            auto &cfeat = this->feat_pts_T.at(s_idx).at(f_idx);
            this->feat_T_map.at(s_idx).at(f_idx) = std::make_shared<Eigen::Map<MatXf>>(cfeat.data(), cfeat.dimension(0), cfeat.dimension(1));
        }
        for (auto &track : this->feature_tracks.at(f_idx)) {
            // plane_cost
            if (f_idx == 2) {
                this->local_params.emplace_back(std::make_shared<PlaneParameterization>());
            } else {
                this->local_params.emplace_back(std::make_shared<LineParameterization>());
            }
            problem.AddParameterBlock(track.geometry.data(), 6, this->local_params.back().get());
            for (uint32_t p_idx = 0; p_idx < track.mapping.size(); ++p_idx) {
                this->loss_functions.emplace_back(new BisquareLoss(this->param.robust_param));
                if (f_idx == 2) {
                    this->costs.emplace_back(
                      new ImplicitPlaneResidual<12, 6, 12, 6>(p_idx, f_idx, &track, &(this->feat_T_map)));
                } else {
                    this->costs.emplace_back(
                      new ImplicitLineResidual<12, 6, 12, 6>(p_idx, f_idx, &track, &(this->feat_T_map)));
                }
                uint32_t start_offset = track.state_ids.at(p_idx);
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
    for (uint32_t state_id = 0; state_id < this->cur_trajectory.size(); ++state_id) {
        auto &state = this->cur_trajectory[state_id];
        this->local_params.emplace_back(std::make_shared<NullSE3Parameterization>());
        problem.AddParameterBlock(state.pose.storage.data(), 12, this->local_params.back().get());
        problem.AddParameterBlock(state.vel.data(), 6);
        if (state_id > 0) {
            //create constant velocity residuals between each state
            auto &pstate = this->cur_trajectory[state_id - 1];
            auto dT = this->trajectory_stamps.at(state_id) - this->trajectory_stamps.at(state_id - 1);
            Mat12 weight;
            this->cv_model->calculateLinInvCovariance(weight, this->trajectory_stamps.at(state_id - 1), this->trajectory_stamps.at(state_id));
            weight = weight.sqrt();
            this->costs.emplace_back(new ConstantVelocityPrior(weight, dT));
            problem.AddResidualBlock(this->costs.back().get(),
                                     nullptr,
                                     pstate.pose.storage.data(),
                                     state.pose.storage.data(),
                                     pstate.vel.data(),
                                     state.vel.data());
        }
    }
    //finally, just fix the first pose
    problem.SetParameterBlockConstant(this->cur_trajectory.front().pose.storage.data());
}

void LaserOdom::resetTrajectory() {
    for (auto tra : this->cur_trajectory) {
        tra.pose.setIdentity();
        tra.vel.setZero();
    }
    this->prior_twist.setZero();
}

}  // namespace wave
