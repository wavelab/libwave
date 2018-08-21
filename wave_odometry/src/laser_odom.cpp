#include "wave/odometry/laser_odom.hpp"

namespace wave {

LaserOdom::LaserOdom(const LaserOdomParams params,
                     const FeatureExtractorParams feat_params,
                     const TransformerParams transformer_params)
    : param(params), feature_extractor(feat_params, params.n_ring), transformer(transformer_params) {
    if (this->param.num_trajectory_states < 2) {
        throw std::out_of_range("Number of parameter states must be at least 2");
    }
    if (this->param.n_window < 2) {
        throw std::out_of_range("Window size must be at least 2");
    }
    this->continue_output = true;

    this->cv_model =
      std::make_shared<wave_kinematics::ConstantVelocityPrior>(0, 0, nullptr, this->param.Qc, this->param.inv_Qc);

    auto n_ring = static_cast<size_t>(param.n_ring);
    this->feature_extractor.setParams(feat_params, n_ring);
    this->counters.resize(n_ring);
    std::fill(this->counters.begin(), this->counters.end(), 0);

    // todo don't do this
    this->cur_feature_candidates.resize(this->N_FEATURES);
    this->cur_feature_candidatesT.resize(this->N_FEATURES);
    this->prev_feature_candidates.resize(this->N_FEATURES);
    this->prev_feature_candidatesT.resize(this->N_FEATURES);
    this->ave_pts.resize(this->N_FEATURES);
    this->feature_tracks.resize(this->N_FEATURES);
    this->cur_feat_idx.resize(this->N_FEATURES);
    this->prev_feat_idx.resize(this->N_FEATURES);
    this->undis_tracks.resize(this->N_FEATURES);
    this->undis_features.resize(this->N_FEATURES);
    this->undis_candidates_cur.resize(this->N_FEATURES);
    this->undis_candidates_prev.resize(this->N_FEATURES);
    this->binner.resize(this->N_FEATURES);
    this->cm1_feat_pts_size.resize(this->N_FEATURES);

    for (auto &biner : this->binner) {
        biner.setAngularBins(this->param.icosahedral_angular_sectors);
    }

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

    this->prior_twist.setZero();
}

void LaserOdom::updateParams(const LaserOdomParams new_params) {
    this->param = new_params;
}

LaserOdomParams LaserOdom::getParams() {
    return this->param;
}

LaserOdom::~LaserOdom() {
    if (this->output_thread) {
        this->continue_output = false;
        this->output_condition.notify_one();
        this->output_thread->join();
    }
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
    MatXf output;
    this->transformer.update(this->cur_trajectory, this->trajectory_stamps);
    this->undistorted_cld.clear();
    for (uint32_t ring_id = 0; ring_id < this->counters.size(); ++ring_id) {
        Eigen::Tensor<float, 2> real_points =
          this->cur_scan.at(ring_id).slice(ar2{0, 0}, ar2{4, this->counters.at(ring_id)});
        this->transformer.transformToStart(real_points, output, this->scan_stampsf.size() - 1);
        for (int i = 0; i < this->counters.at(ring_id); ++i) {
            pcl::PointXYZI pt;
            pt.x = output(0, i);
            pt.y = output(1, i);
            pt.z = output(2, i);
            pt.intensity = this->signals.at(ring_id)(1, i);
            this->undistorted_cld.push_back(pt);
        }
    }
    for (uint32_t feat_id = 0; feat_id < this->N_FEATURES; ++feat_id) {
        this->undis_tracks.at(feat_id) = this->feature_tracks.at(feat_id);
        this->undis_features.at(feat_id).clear();
        auto n_scans = this->feat_pts_T.size();
        for (uint32_t i = 0; i < n_scans; ++i) {
            for (uint32_t j = 0; j < this->feat_pts_T.at(i).at(feat_id).cols(); ++j) {
                pcl::PointXYZ pt;
                pt.x = this->feat_pts_T.at(i).at(feat_id)(0, j);
                pt.y = this->feat_pts_T.at(i).at(feat_id)(1, j);
                pt.z = this->feat_pts_T.at(i).at(feat_id)(2, j);
                this->undis_features.at(feat_id).push_back(pt);
            }
        }
        this->undis_candidates_prev.at(feat_id).clear();
        this->undis_candidates_cur.at(feat_id).clear();
        for (uint32_t i = 0; i < this->cur_feature_candidatesT.at(feat_id).cols(); ++i) {
            pcl::PointXYZ pt;
            pt.x = this->cur_feature_candidatesT.at(feat_id)(0, i);
            pt.y = this->cur_feature_candidatesT.at(feat_id)(1, i);
            pt.z = this->cur_feature_candidatesT.at(feat_id)(2, i);
            this->undis_candidates_cur.at(feat_id).push_back(pt);
        }
        for (uint32_t i = 0; i < this->prev_feature_candidatesT.at(feat_id).cols(); ++i) {
            pcl::PointXYZ pt;
            pt.x = this->prev_feature_candidatesT.at(feat_id)(0, i);
            pt.y = this->prev_feature_candidatesT.at(feat_id)(1, i);
            pt.z = this->prev_feature_candidatesT.at(feat_id)(2, i);
            this->undis_candidates_prev.at(feat_id).push_back(pt);
        }
    }
}

template <class S_TYPE, class D_TYPE>
void LaserOdom::copyTrajectory(const VecE<S_TYPE> &src, VecE<D_TYPE> &dst) {
    dst.resize(src.size());
    for (uint32_t i = 0; i < src.size(); i++) {
        dst.at(i).pose = src.at(i).pose;
        dst.at(i).vel = src.at(i).vel;
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
            // can't use omp parallel for because of offset. Need to manually split up task to parallelize.
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

            if (this->output_thread) {
                {
                    std::unique_lock<std::mutex> lk(this->output_mutex);
                    if (this->fresh_output) {
                        // data from last time hasn't been consumed yet
                        LOG_ERROR("Overwriting previous output");
                    }
                    this->copyTrajectory(this->cur_trajectory, this->undistort_trajectory);
                    for (uint32_t i = 0; i < this->undistort_trajectory.size(); ++i) {
                        std::chrono::duration<float> fsec(this->trajectory_stamps.at(i));
                        this->undistort_trajectory.at(i).stamp =
                          this->scan_stamps_chrono.front() +
                          std::chrono::duration_cast<std::chrono::microseconds>(fsec);
                    }

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

        if (secon < 0.0f) {
            secon = 0;
        }

        this->cur_scan.at(pt.ring)(3, counters.at(pt.ring)) = secon;

        this->signals.at(pt.ring)(0, counters.at(pt.ring)) = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        this->signals.at(pt.ring)(1, counters.at(pt.ring)) = pt.intensity;

        this->counters.at(pt.ring)++;
    }

    this->prv_tick = tick;
}

void LaserOdom::updateTracks() {
    // transform all landmark states to start of current scan and decrement the scan id of each associated feature point
    const auto &transform = this->cur_trajectory.at(this->param.num_trajectory_states - 1).pose;
    for (auto &tracks : this->feature_tracks) {
        VecE<FeatureTrack> updated_tracks;
        for (auto &track : tracks) {
            bool reduce_length = false;
            Vec<FeatureTrack::Mapping> updated_mapping;
            for (auto &map : track.mapping) {
                if (map.scan_idx != 0) {
                    --(map.scan_idx);
                    map.state_id = map.state_id + 1 - this->param.num_trajectory_states;
                    updated_mapping.emplace_back(map);
                } else {
                    reduce_length = true;
                }
            }
            if (updated_mapping.empty()) {
                continue;
            }
            std::swap(track.mapping, updated_mapping);

            if (reduce_length) {
                --(track.length);
            }
            track.geometry.block<3, 1>(0, 0) =
              transform.storage.block<3, 3>(0, 0).transpose() * track.geometry.block<3, 1>(0, 0);
            auto ref = track.geometry.block<3, 1>(3, 0);
            transform.inverseTransform(ref, ref);
            updated_tracks.emplace_back(std::move(track));
        }
        std::swap(tracks, updated_tracks);
    }
}

void LaserOdom::rollover(TimeType stamp) {
    // If there are n_scans worth of data, rotate
    if (this->scan_stamps_chrono.size() == this->param.n_window) {
        this->prior_twist = this->cur_trajectory.front().vel;
        this->prev_delta_t = (double) this->scan_stampsf.at(1);
        // Perform a left rotation
        this->updateTracks();
        std::rotate(this->feat_pts.begin(), this->feat_pts.begin() + 1, this->feat_pts.end());
        std::rotate(this->feat_pts_T.begin(), this->feat_pts_T.begin() + 1, this->feat_pts_T.end());
        std::rotate(
          this->scan_stamps_chrono.begin(), this->scan_stamps_chrono.begin() + 1, this->scan_stamps_chrono.end());

        std::rotate(this->cur_trajectory.begin(),
                    this->cur_trajectory.begin() + this->param.num_trajectory_states - 1,
                    this->cur_trajectory.end());

        // adjust timestamps and trajectory states to start of new window
        for (uint32_t i = 1; i < this->cur_trajectory.size(); ++i) {
            this->cur_trajectory.at(i).pose =
              this->cur_trajectory.front().pose.transformInverse() * this->cur_trajectory.at(i).pose;
        }
        this->cur_trajectory.front().pose.setIdentity();
        this->scan_stamps_chrono.back() = stamp;

    } else {
        // grow storage
        this->scan_stamps_chrono.emplace_back(stamp);
        this->scan_stampsf.resize(this->scan_stamps_chrono.size());
        VecE<Eigen::Tensor<float, 2>> vec(this->N_FEATURES);
        VecE<MatXf> vec2(this->N_FEATURES);

        for (auto &elem : vec) {
            elem.resize(4, 0);
        }

        this->feat_pts.emplace_back(std::move(vec));
        this->feat_pts_T.emplace_back(std::move(vec2));

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

    this->skip_point.resize(this->scan_stampsf.size());
    for (auto &array : this->skip_point) {
        array.resize(this->N_FEATURES);
    }

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

bool LaserOdom::runOptimization(ceres::Problem &problem, ceres::Solver::Summary &summary) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.use_explicit_schur_complement = true;
    options.max_num_iterations = this->param.max_inner_iters;
    options.max_num_consecutive_invalid_steps = 30;
    options.logging_type = ceres::LoggingType::SILENT;
    options.use_nonmonotonic_steps = true;

    ceres::EvaluationCallback *callback = new OdometryCallback(&(this->feat_pts),
                                                               &(this->feat_pts_T),
                                                               &(this->cur_trajectory),
                                                               &(this->ptT_jacobians),
                                                               &(this->jacobian_stamps),
                                                               &(this->trajectory_stamps),
                                                               &(this->scan_stampsf),
                                                               &(this->transformer),
                                                               &(this->skip_point));
    options.evaluation_callback = callback;

    this->buildResiduals(problem);

    if (this->param.solver_threads < 1) {
        options.num_threads = std::thread::hardware_concurrency();
        options.num_linear_solver_threads = std::thread::hardware_concurrency();
    } else {
        options.num_threads = this->param.solver_threads;
        options.num_linear_solver_threads = this->param.solver_threads;
    }

    if (problem.NumResidualBlocks() < this->param.min_residuals) {
        LOG_ERROR("Less than expected residuals, resetting");
        LOG_ERROR("%d residuals, threshold is %d", problem.NumResidualBlocks(), this->param.min_residuals);
        this->resetTrajectory();
        this->initialized = false;
        return false;
    }
    ceres::Solve(options, &problem, &summary);
    if (this->param.print_opt_sum) {
        LOG_INFO("%s", summary.BriefReport().c_str());
    }

    return true;
}

void LaserOdom::calculateCovariance(ceres::Problem &problem) {
    ceres::Covariance::Options covar_options;
    covar_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
    covar_options.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
    covar_options.null_space_rank = -1;

    if (this->param.solver_threads < 1) {
        covar_options.num_threads = std::thread::hardware_concurrency();
    } else {
        covar_options.num_threads = this->param.solver_threads;
    }

    ceres::Covariance covariance(covar_options);
    std::vector<const double *> blocks;
    blocks.emplace_back(this->cur_trajectory.front().vel.data());
    if (!covariance.Compute(blocks, &problem)) {
        LOG_ERROR("covariance did not compute");
    } else {
        covariance.GetCovarianceMatrix(blocks, this->twist_covar.data());
    }
}

template <typename Derived, typename Derived1, typename Derived2>
bool LaserOdom::findLineCorrespondences(std::vector<uint32_t> &matches,
                                        std::vector<int> &used_points,
                                        const Eigen::MatrixBase<Derived> &index,
                                        const Eigen::MatrixBase<Derived1> &distances,
                                        const Eigen::MatrixBase<Derived2> &points) {
    matches.clear();
    double min_elev = 0.0;
    double max_elev = 0.0;
    double new_min = 0.0, new_max = 0.0;
    bool wide_spread = false;
    bool new_spread = false;
    /// Each row is a nearest neighbour
    for (uint32_t i = 0; i < index.rows(); ++i) {
        if (std::isinf(distances(i))) {
            break;
        }
        if (used_points.at(index(i)) != -1) {
            continue;
        }
        Vec3f pt = points.template block<3, 1>(0, index(i));
        double xydist = std::sqrt(pt(0) * pt(0) + pt(1) * pt(1));
        double range = std::sqrt(xydist * xydist + pt(2) * pt(2));
        double scaled_elev_tol = this->param.elevation_tol * (10.0 / range);
        if (matches.empty()) {
            matches.emplace_back(index(i));
            min_elev = std::atan2(pt(2), xydist);
            max_elev = min_elev;
            continue;
        }
        if (wide_spread) {
            matches.emplace_back(index(i));
        } else {
            double new_elev = std::atan2(pt(2), xydist);
            if (new_elev > max_elev) {
                new_max = new_elev;
                new_min = min_elev;
            } else if (new_elev < min_elev) {
                new_min = new_elev;
                new_max = max_elev;
            }
            if (new_max - new_min > scaled_elev_tol)
                new_spread = true;
            if (new_spread) {
                wide_spread = true;
                min_elev = new_min;
                max_elev = new_max;
            }
            if (wide_spread || matches.size() + 1 < 2) {
                matches.emplace_back(index(i));
            }
        }
        if (matches.size() == 2) {
            return true;
        }
    }
    return false;
}

template <typename Derived, typename Derived1, typename Derived2>
bool LaserOdom::findPlaneCorrespondences(std::vector<uint32_t> &matches,
                                         Vec<int> &used_points,
                                         const Eigen::MatrixBase<Derived> &index,
                                         const Eigen::MatrixBase<Derived1> &distances,
                                         const Eigen::MatrixBase<Derived2> &points) {
    matches.clear();
    double min_elev = 0.0;
    double max_elev = 0.0;
    double new_min = 0.0, new_max = 0.0;
    bool wide_spread = false;
    bool new_spread = false;
    /// Each row is a nearest neighbour
    for (uint32_t i = 0; i < index.rows(); ++i) {
        if (std::isinf(distances(i))) {
            break;
        }
        if (used_points.at(index(i)) != -1) {
            continue;
        }
        Vec3f pt = points.template block<3, 1>(0, index(i));
        double xydist = std::sqrt(pt(0) * pt(0) + pt(1) * pt(1));
        double range = std::sqrt(xydist * xydist + pt(2) * pt(2));
        double scaled_elev_tol = this->param.elevation_tol * (10.0 / range);
        if (matches.empty()) {
            matches.emplace_back(index(i));
            min_elev = std::atan2(pt(2), xydist);
            max_elev = min_elev;
            continue;
        }
        // check to make sure that this point is not colinear with the other two
        if (matches.size() + 1 == 3) {
            auto ptA = points.template block<3, 1>(0, matches.at(0));
            auto ptB = points.template block<3, 1>(0, matches.at(1));
            Vec3f AmB = ptA - ptB;
            Vec3f AmO = ptA - pt;
            AmB.normalize();
            AmO.normalize();
            if (std::abs((AmB.transpose() * AmO)(0)) > 0.7) {
                continue;
            }
        }
        if (wide_spread) {
            matches.emplace_back(index(i));
        } else {
            double new_elev = std::atan2(pt(2), xydist);
            if (new_elev > max_elev) {
                new_max = new_elev;
                new_min = min_elev;
            } else if (new_elev < min_elev) {
                new_min = new_elev;
                new_max = max_elev;
            }
            if (new_max - new_min > scaled_elev_tol)
                new_spread = true;
            if (new_spread) {
                wide_spread = true;
                min_elev = new_min;
                max_elev = new_max;
            }
            if (wide_spread || matches.size() + 1 < 3) {
                matches.emplace_back(index(i));
            }
        }
        if (matches.size() == 3) {
            return true;
        }
    }
    return false;
}

void LaserOdom::extendFeatureTracks(const Eigen::MatrixXi &idx, const MatXf &distances, uint32_t feat_id,
                                    const bool large_tol) {
    auto residual_type = PointToLine;
    double max_residual_val = this->param.max_linear_residual_val;
    // todo don't do this
    if (feat_id == 2) {
        residual_type = PointToPlane;
        max_residual_val = this->param.max_planar_residual_val;
    }
    if (large_tol) {
        max_residual_val = 1;
    }
    max_residual_val *= max_residual_val;
    /// Each column represents a query (feature track)
    Eigen::Tensor<float, 2> new_feat_points(4, 2000);
    auto offset = this->feat_pts.back().at(feat_id).dimension(1);
    long new_feat_cnt = 0;
    for (uint32_t j = 0; j < idx.cols(); ++j) {
        /// add to track if error against current landmark is low
        auto &track = this->feature_tracks.at(feat_id).at(j);
        if (!track.optimize) {
            continue;
        }
        const auto &geometry = track.geometry;
        uint32_t new_points = 0;
        for (int32_t k = 0; k < idx.rows(); ++k) {
            if (!std::isinf(distances(k, j))) {
                const auto &pt = this->cur_feature_candidatesT.at(feat_id).block<3, 1>(0, idx(k, j));
                Vec3 diff = (pt.cast<double>() - geometry.block<3, 1>(3, 0));
                Vec1 error;
                if (residual_type == PointToLine) {
                    Vec3 err = (diff - geometry.block<3, 1>(0, 0) * (diff.transpose() * geometry.block<3, 1>(0, 0)));
                    error = err.transpose() * err;
                } else {
                    error = (diff.transpose() * geometry.block<3, 1>(0, 0));
                    error = error.transpose() * error;
                }

                // copy candidate point into the feature point set (both original and transformed) if it is not already
                // used in another residual
                if (error(0) < max_residual_val) {
                    this->cur_feat_idx.at(feat_id).at(idx(k, j)) = j;

                    Eigen::array<int, 2> offsets_new = {0, static_cast<int>(new_feat_cnt)};
                    Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(idx(k, j))};
                    Eigen::array<int, 2> extents = {4, 1};

                    new_feat_points.slice(offsets_new, extents) =
                            this->cur_feature_candidates.at(feat_id).slice(offsets_candidate, extents);

                    auto new_scan_idx = this->scan_stampsf.size() - 1;
                    if (this->feature_tracks.at(feat_id).at(j).mapping.back().scan_idx != new_scan_idx) {
                        this->feature_tracks.at(feat_id).at(j).length += 1;
                    }
                    this->feature_tracks.at(feat_id).at(j).mapping.emplace_back(offset + new_feat_cnt, new_scan_idx);
                    auto bnd = std::upper_bound(this->trajectory_stamps.begin(),
                                                this->trajectory_stamps.end(),
                                                new_feat_points(3, new_feat_cnt) + this->scan_stampsf.back());
                    auto traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);

                    this->feature_tracks.at(feat_id).at(j).mapping.back().state_id = traj_idx;

                    ++new_feat_cnt;
                    ++new_points;
                }
            }
        }
        if (new_points < this->param.min_new_points) {
            track.optimize = false;
            this->binner.at(feat_id).deBin(track.geometry);
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

void LaserOdom::createNewGeometry(const MatXf &points,
                                  const Vec<uint32_t> &indices,
                                  ResidualType residual_type,
                                  Vec6 &geometry) {
    if (residual_type == PointToLine) {
        geometry.block<3, 1>(3, 0) =
          (points.block<3, 1>(0, indices.at(0)).cast<double>() + points.block<3, 1>(0, indices.at(1)).cast<double>()) /
          2.0;

        geometry.block<3, 1>(0, 0) =
          points.block<3, 1>(0, indices.at(0)).cast<double>() - points.block<3, 1>(0, indices.at(1)).cast<double>();
    } else {
        geometry.block<3, 1>(3, 0) =
          (points.block<3, 1>(0, indices.at(0)).cast<double>() + points.block<3, 1>(0, indices.at(1)).cast<double>() +
           points.block<3, 1>(0, indices.at(2)).cast<double>()) /
          3.0;

        Vec3f a = points.block<3, 1>(0, indices.at(0)) - points.block<3, 1>(0, indices.at(1));
        Vec3f b = points.block<3, 1>(0, indices.at(0)) - points.block<3, 1>(0, indices.at(2));
        geometry.block<3, 1>(0, 0) = a.cross(b).cast<double>();
    }
    geometry.block<3, 1>(0, 0).normalize();
    if (geometry(2) < 0) {
        geometry.block<3, 1>(0, 0) = -geometry.block<3, 1>(0, 0);
    }
}
// todo reduce duplicated code
void LaserOdom::createNewFeatureTracks(const Eigen::MatrixXi &idx, const MatXf &distances, uint32_t feat_id,
                                       const Nabo::NNSearchF *cur_knn_tree, const bool large_tol) {
    auto residual_type = PointToLine;
    double max_residual_val = this->param.max_linear_residual_val;
    // todo don't do this
    if (feat_id == 2) {
        residual_type = PointToPlane;
        max_residual_val = this->param.max_planar_residual_val;
    }
    if (large_tol) {
        max_residual_val = 1;
    }
    max_residual_val *= max_residual_val;
    /// Each column represents a query (feature track)
    Eigen::Tensor<float, 2> new_feat_points(4, 3000), prev_new_feat_points(4, 3000);
    long new_feat_cnt = 0, prev_new_feat_cnt = 0;
    VecE<FeatureTrack> candidate_tracks;
    MatXf query(3, 3000);
    long candidate_track_cnt = 0;
    auto &prev_feat_points = this->feat_pts.at(this->feat_pts.size() - 2).at(feat_id);
    auto &cur_feat_points = this->feat_pts.at(this->feat_pts.size() - 1).at(feat_id);
    auto prev_offset = prev_feat_points.dimension(1);
    auto cur_offset = cur_feat_points.dimension(1);
    bool success;
    std::vector<uint32_t> matches;
    for (uint32_t j = 0; j < idx.cols(); ++j) {
        if (this->cur_feat_idx.at(feat_id).at(j) != -1) {
            continue;
        }

        if (residual_type == PointToLine) {
            success = this->findLineCorrespondences(matches,
                                                    this->prev_feat_idx.at(feat_id),
                                                    idx.col(j),
                                                    distances.col(j),
                                                    this->prev_feature_candidatesT.at(feat_id));
        } else {
            success = this->findPlaneCorrespondences(matches,
                                                     this->prev_feat_idx.at(feat_id),
                                                     idx.col(j),
                                                     distances.col(j),
                                                     this->prev_feature_candidatesT.at(feat_id));
        }

        /// create feature track if error is low, and points are not part of another track
        if (success) {
            // Depending on the type of geometry, need to initialize feature track with reasonable estimate
            Vec6 geometry;
            this->createNewGeometry(this->prev_feature_candidatesT.at(feat_id), matches, residual_type, geometry);

            Vec3 diff =
              (this->cur_feature_candidatesT.at(feat_id).block<3, 1>(0, j).cast<double>() - geometry.block<3, 1>(3, 0));
            Vec1 error;
            if (residual_type == PointToLine) {
                Vec3 err = (diff - geometry.block<3, 1>(0, 0) * (diff.transpose() * geometry.block<3, 1>(0, 0)));
                error = err.transpose() * err;
            } else {
                error = (diff.transpose() * geometry.block<3, 1>(0, 0));
                error = error.transpose() * error;
            }

            // create new residual, may need to increase tolerance a bit to find new features when initializing
            if (error(0) < max_residual_val) {
                //                query.block<3, 1>(0, candidate_track_cnt) = (0.5f * geometry.block<3, 1>(3,
                //                0).cast<float>() + 0.5f * this->cur_feat_map.at(feat_id)->block<3, 1>(0, j));
                query.block<3, 1>(0, candidate_track_cnt) = this->cur_feature_candidatesT.at(feat_id).block<3, 1>(0, j);
                ++candidate_track_cnt;
                // add to candidate feature tracks
                candidate_tracks.emplace_back(FeatureTrack());
                auto &track = candidate_tracks.back();
                track.geometry = geometry;
                track.length = 1;

                // add points from previous scan
                for (auto elem : matches) {
                    track.mapping.emplace_back(elem, this->scan_stampsf.size() - 2);
                }
                // add point from current scan
                track.mapping.emplace_back(j, this->scan_stampsf.size() - 1);
            }
        }
    }

    // now search current feature candidates with candidate feature tracks
    query.conservativeResize(Eigen::NoChange, candidate_track_cnt);
    MatXf dists;
    Eigen::MatrixXi nn_idx;
    dists.resize(5, candidate_track_cnt);
    nn_idx.resize(5, candidate_track_cnt);
    cur_knn_tree->knn(query,
                      nn_idx,
                      dists,
                      5,
                      0,
                      Nabo::NNSearchF::SORT_RESULTS | Nabo::NNSearchF::ALLOW_SELF_MATCH,
                      this->param.max_correspondence_dist);

    for (long i = 0; i < candidate_track_cnt; ++i) {
        auto &candidate_track = candidate_tracks.at(static_cast<unsigned long>(i));
        if (residual_type == PointToLine) {
            success = this->findLineCorrespondences(matches,
                                                    this->cur_feat_idx.at(feat_id),
                                                    nn_idx.col(i),
                                                    dists.col(i),
                                                    this->cur_feature_candidatesT.at(feat_id));
        } else {
            success = this->findPlaneCorrespondences(matches,
                                                     this->cur_feat_idx.at(feat_id),
                                                     nn_idx.col(i),
                                                     dists.col(i),
                                                     this->cur_feature_candidatesT.at(feat_id));
        }
        if (success) {
            Vec6 geometry;
            this->createNewGeometry(this->cur_feature_candidatesT.at(feat_id), matches, residual_type, geometry);
            bool add_track = false;
            float dist_cost, dir_cost;
            if (residual_type == PointToLine) {
                this->calculateLineSimilarity(geometry, candidate_track.geometry, dist_cost, dir_cost);
                if (dist_cost < this->param.max_linear_dist_threshold &&
                    dir_cost < this->param.max_linear_ang_threshold) {
                    add_track = true;
                }
            } else {
                this->calculatePlaneSimilarity(geometry, candidate_track.geometry, dist_cost, dir_cost);
                if (dist_cost < this->param.max_planar_dist_threshold &&
                    dir_cost < this->param.max_planar_ang_threshold) {
                    add_track = true;
                }
            }
            if (add_track) {
                for (const auto &match : matches) {
                    candidate_track.mapping.emplace_back(match, this->scan_stampsf.size() - 1);
                }
            }
        }
    }

    VecE<FeatureTrack> shortlist;
    for (const auto &candidate_track : candidate_tracks) {
        if (candidate_track.mapping.size() > 4) {
            shortlist.emplace_back(candidate_track);
        }
    }

    //merge shortlist and sort by number of measurements
    this->mergeFeatureTracks(shortlist, feat_id);

    std::sort(shortlist.begin(), shortlist.end(), [](const FeatureTrack &lhs, const FeatureTrack &rhs) {
        return lhs.mapping.size() > rhs.mapping.size();
    });

    // For any successes, add to feature points and to feature tracks
    for (const auto &candidate_track : shortlist) {
        if (this->binner.at(feat_id).bin(candidate_track.geometry, this->param.icosahedral_bin_limit)) {
            this->feature_tracks.at(feat_id).emplace_back(candidate_track);
            auto &track_ref = this->feature_tracks.at(feat_id).back();

            for (auto &map : track_ref.mapping) {
                if (map.scan_idx + 1 == this->scan_stampsf.size()) {
                    Eigen::array<int, 2> offsets_new = {0, static_cast<int>(new_feat_cnt)};
                    Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(map.pt_idx)};
                    Eigen::array<int, 2> extents = {4, 1};

                    new_feat_points.slice(offsets_new, extents) =
                      this->cur_feature_candidates.at(feat_id).slice(offsets_candidate, extents);

                    this->cur_feat_idx.at(feat_id).at(map.pt_idx) =
                      static_cast<int>(this->feature_tracks.at(feat_id).size() - 1);
                    map.pt_idx = static_cast<uint32_t>(new_feat_cnt + cur_offset);

                    auto bnd = std::upper_bound(this->trajectory_stamps.begin(),
                                                this->trajectory_stamps.end(),
                                                new_feat_points(3, new_feat_cnt) + this->scan_stampsf.back());
                    auto traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);
                    map.state_id = traj_idx;

                    ++new_feat_cnt;
                } else {
                    Eigen::array<int, 2> offsets_new = {0, static_cast<int>(prev_new_feat_cnt)};
                    Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(map.pt_idx)};
                    Eigen::array<int, 2> extents = {4, 1};

                    prev_new_feat_points.slice(offsets_new, extents) =
                      this->prev_feature_candidates.at(feat_id).slice(offsets_candidate, extents);
                    this->prev_feat_idx.at(feat_id).at(map.pt_idx) =
                      static_cast<int>(this->feature_tracks.at(feat_id).size() - 1);
                    map.pt_idx = static_cast<uint32_t>(prev_new_feat_cnt + prev_offset);

                    float pttime =
                      prev_new_feat_points(3, prev_new_feat_cnt) + this->scan_stampsf.at(this->scan_stampsf.size() - 2);
                    auto bnd = std::upper_bound(this->trajectory_stamps.begin(), this->trajectory_stamps.end(), pttime);
                    auto traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);
                    map.state_id = traj_idx;

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

    Eigen::TensorMap<Eigen::Tensor<float, 2>> reduced_prev_new_feat_points(
      prev_new_feat_points.data(), 4, prev_new_feat_cnt);
    if (prev_feat_points.dimension(1) == 0) {
        prev_feat_points = reduced_prev_new_feat_points;
    } else if (reduced_prev_new_feat_points.dimension(1) != 0) {
        Eigen::Tensor<float, 2> stupid_eigen = prev_feat_points.concatenate(reduced_prev_new_feat_points, 1);
        prev_feat_points = stupid_eigen;
    }
}

void LaserOdom::clearVolatileTracks() {
    auto second_last_scan = this->feat_pts.size() - 2;
    for (uint32_t feat_id = 0; feat_id < this->feature_tracks.size(); ++feat_id) {
        this->binner.at(feat_id).clear();
        auto &tracks = this->feature_tracks.at(feat_id);
        for (int i = 0; i < static_cast<int>(tracks.size()); ++i) {
            auto &track = tracks.at(i);
            auto iter = track.mapping.rbegin();
            // If the only references are to scans older than the two most recent, there is no chance that
            // it may contain volatile feature points.
            if (iter->scan_idx < second_last_scan) {
                continue;
            }

            long dist = 0;
            uint32_t num_prev_points = 0;
            while (iter != track.mapping.rend() && iter->scan_idx >= second_last_scan) {
                // this point is from the current scan, get rid of it
                if (iter->scan_idx > second_last_scan) {
                    ++dist;
                } else {
                    // the point is from the previous scan, if it was new, get rid of it
                    if (iter->pt_idx >= this->cm1_feat_pts_size.at(feat_id)) {
                        ++dist;
                    }
                    ++num_prev_points;
                }
                ++iter;
            }

            unsigned long new_size = track.mapping.size() - dist;
            track.mapping.resize(new_size);
            track.length = track.mapping.back().scan_idx - track.mapping.front().scan_idx;

            if (num_prev_points >= this->param.min_new_points) {
                track.optimize = true;
            }

            // If the length is zero, remove the track
            if (track.length == 0 || track.mapping.empty() || !this->binner.at(feat_id).bin(track.geometry, this->param.icosahedral_bin_limit)) {
                if (i + 1 != static_cast<int>(tracks.size())) {
                    std::swap(track, tracks.at(tracks.size() - 1));
                    --i;
                }
                tracks.resize(tracks.size() - 1);
            }
        }
    }
    for (uint32_t feat_id = 0; feat_id < this->feat_pts.back().size(); ++feat_id) {
        auto &pts_cur = this->feat_pts.back().at(feat_id);
        pts_cur.resize(pts_cur.dimension(0), 0);

        auto &pts_prev = (this->feat_pts.rbegin() + 1)->at(feat_id);
        Eigen::Tensor<float, 2> temp = pts_prev.slice(ar2{0, 0}, ar2{4, this->cm1_feat_pts_size.at(feat_id)});
        std::swap(temp, pts_prev);
    }
}

void LaserOdom::calculateLineSimilarity(const wave::Vec6 &geo1,
                                        const wave::Vec6 &geo2,
                                        float &dist_cost,
                                        float &dir_cost) {
    Vec3f diff = geo1.block<3, 1>(3, 0).cast<float>() - geo2.block<3, 1>(3, 0).cast<float>();
    dir_cost = 1.0f - std::abs((geo1.block<3, 1>(0, 0).transpose() * geo2.block<3, 1>(0, 0))(0));

    Vec3f cross = geo1.block<3, 1>(0, 0).cross(geo2.block<3, 1>(0, 0)).cast<float>();

    dist_cost = (diff.transpose() * cross).norm() / cross.norm();
}

void LaserOdom::calculatePlaneSimilarity(const wave::Vec6 &geo1,
                                         const wave::Vec6 &geo2,
                                         float &dist_cost,
                                         float &dir_cost) {
    Vec3f diff = geo1.block<3, 1>(3, 0).cast<float>() - geo2.block<3, 1>(3, 0).cast<float>();
    dir_cost = 1.0f - std::abs((geo1.block<3, 1>(0, 0).transpose() * geo2.block<3, 1>(0, 0))(0));

    dist_cost = std::abs((diff.transpose() * geo1.block<3, 1>(0, 0).cast<float>())) +
                std::abs((diff.transpose() * geo2.block<3, 1>(0, 0).cast<float>()));
}

void LaserOdom::mergeFeatureTracks(VecE <FeatureTrack> &tracks, uint32_t feat_id) {
    if (tracks.size() <= 1) {
        return;
    }
    MatXf trk_pts;
    trk_pts.resize(3, tracks.size());

    const int knn = 10 < (tracks.size() - 1) ? 10 : (tracks.size() - 1);

    Vec<std::list<uint32_t>> merge_history(tracks.size());
    Vec<uint32_t> addr(tracks.size());
    std::iota(addr.begin(), addr.end(), 0);

//#pragma omp parallel for
    for (uint32_t t_idx = 0; t_idx < tracks.size(); t_idx++) {
        merge_history.at(t_idx).emplace_back(t_idx);
        const auto &track = tracks.at(t_idx);
        trk_pts.block<3, 1>(0, t_idx) = track.geometry.block<3, 1>(3, 0).cast<float>();
    }

    Nabo::NNSearchF *kd_idx = Nabo::NNSearchF::createKDTreeLinearHeap(trk_pts);

    MatXf dists(knn, trk_pts.cols());
    Eigen::MatrixXi nn_idx(knn, trk_pts.cols());

    kd_idx->knn(trk_pts, nn_idx, dists, knn, 0.1, 0, this->param.max_correspondence_dist);

    std::vector<float> score(knn);
    for (uint32_t c_idx = 0; c_idx < nn_idx.cols(); ++c_idx) {
        uint32_t query_idx = addr.at(c_idx);
        auto &query_track = tracks.at(query_idx);
        std::fill(score.begin(), score.end(), std::numeric_limits<float>::infinity());
        for (uint32_t r_idx = 0; r_idx < nn_idx.rows(); ++r_idx) {
            // calculate similarity score to each correspondence

            if (std::isinf(dists(r_idx, c_idx))) {
                continue;
            }
            float dist_cost, dir_cost;

            uint32_t knn_index = addr.at(nn_idx(r_idx, c_idx));

            auto &knn_track = tracks.at(knn_index);
            // use planar similarity score
            if (feat_id == 2) {
                this->calculatePlaneSimilarity(query_track.geometry, knn_track.geometry, dist_cost, dir_cost);
                if (dist_cost > this->param.max_planar_dist_threshold ||
                    dir_cost > this->param.max_planar_ang_threshold) {
                    continue;
                }
            } else {
                this->calculateLineSimilarity(query_track.geometry, knn_track.geometry, dist_cost, dir_cost);
                if (dist_cost > this->param.max_linear_dist_threshold ||
                    dir_cost > this->param.max_linear_ang_threshold) {
                    continue;
                }
            }
            score.at(r_idx) = dist_cost + this->param.ang_scaling_param * dir_cost;
        }
        auto merge = std::min_element(score.begin(), score.end()) - score.begin();
        if (merge == knn || std::isinf(score.at(merge))) {
            continue;
        } else {
            // merge the two feature tracks
            auto first_index = addr.at(c_idx);
            auto second_index = addr.at(nn_idx(merge, c_idx));

            // This would prevent merging of tracks that have already been merged.
            //            if (first_index != c_idx || second_index != nn_idx(merge, c_idx)) {
            //                continue;
            //            }

            if (first_index == second_index) {
                continue;
            }

            if (first_index > second_index) {
                std::swap(first_index, second_index);
            }

            auto &merged_track = tracks.at(second_index);
            auto total = query_track.mapping.size() + merged_track.mapping.size();
            float w1 = (float) (query_track.mapping.size()) / (float) (total);
            float w2 = (float) (merged_track.mapping.size()) / (float) (total);

            if ((query_track.geometry.block<3, 1>(0, 0).transpose() * merged_track.geometry.block<3, 1>(0, 0))(0) < 0) {
                query_track.geometry.block<3, 1>(3, 0) =
                  w1 * query_track.geometry.block<3, 1>(3, 0) + w2 * merged_track.geometry.block<3, 1>(3, 0);
                query_track.geometry.block<3, 1>(0, 0) =
                  w1 * query_track.geometry.block<3, 1>(0, 0) - w2 * merged_track.geometry.block<3, 1>(0, 0);
            } else {
                query_track.geometry = w1 * query_track.geometry + w2 * merged_track.geometry;
            }

            query_track.geometry.block<3, 1>(0, 0).normalize();
            for (const auto &map : merged_track.mapping) {
                query_track.mapping.emplace_back(map);
            }
            std::sort(query_track.mapping.begin(),
                      query_track.mapping.end(),
                      [](const FeatureTrack::Mapping &lhs, const FeatureTrack::Mapping &rhs) {
                          return lhs.scan_idx < rhs.scan_idx;
                      });
            query_track.length = query_track.mapping.back().scan_idx - query_track.mapping.front().scan_idx;

            // update address map
            merge_history.at(first_index).splice(merge_history.at(first_index).end(), merge_history.at(second_index));

            for (const auto &elem : merge_history.at(first_index)) {
                addr.at(elem) = first_index;
            }

            if (second_index != merge_history.size() - 1) {
                std::swap(merge_history.at(second_index), merge_history.back());
                std::swap(tracks.at(second_index), tracks.back());
                for (const auto &elem : merge_history.at(second_index)) {
                    addr.at(elem) = second_index;
                }
            }

            merge_history.resize(tracks.size() - 1);
            tracks.resize(tracks.size() - 1);
        }
    }
    delete kd_idx;
}

void LaserOdom::setupSkipPoint(uint32_t feat_id) {
    for (uint32_t i = 0; i < this->feat_pts.size(); ++i) {
        this->skip_point.at(i).at(feat_id).resize(this->feat_pts.at(i).at(feat_id).size());
        std::fill(this->skip_point.at(i).at(feat_id).begin(), this->skip_point.at(i).at(feat_id).end(), true);
    }
    for (const auto &track : this->feature_tracks.at(feat_id)) {
        for (const auto &map : track.mapping) {
            this->skip_point.at(map.scan_idx).at(feat_id).at(map.pt_idx) = false;
        }
    }
}

void LaserOdom::prepTrajectory(const TimeType &stamp) {
    // scan stamps f holds the timestamps at the start of each scan
    // need to update the latter half of trajectory stamps based on the difference between
    // stamp and the last timestamp in stamps chrono.
    auto diff = stamp - this->scan_stamps_chrono.back();
    float scan_duration = std::chrono::duration<float, std::ratio<1>>(diff).count();

    // set up timestamps and trajectory for current scan
    float step = scan_duration / (this->param.num_trajectory_states - 1);

    unlong idx = this->trajectory_stamps.size() - this->param.num_trajectory_states + 1;
    for (; idx < this->trajectory_stamps.size(); ++idx) {
        this->trajectory_stamps.at(idx) = this->trajectory_stamps.at(idx - 1) + step;

        this->cur_trajectory.at(idx).pose = this->cur_trajectory.at(idx - 1).pose;
        this->cur_trajectory.at(idx).pose.manifoldPlus(step * this->cur_trajectory.at(idx - 1).vel);
        this->cur_trajectory.at(idx).vel = this->cur_trajectory.at(idx - 1).vel;
    }
    // as to limit floating point error
    this->trajectory_stamps.back() =
      std::chrono::duration<float, std::ratio<1>>(stamp - this->scan_stamps_chrono.front()).count();

    // Now previous trajectory will hold the "motion generated" trajectory
    this->copyTrajectory(this->cur_trajectory, this->prev_trajectory);
}

bool LaserOdom::match(const TimeType &stamp) {
    this->prepTrajectory(stamp);

    std::unique_ptr<ceres::Problem> problem;

    T_TYPE last_transform;
    auto &ref = this->cur_trajectory.back().pose;

    Vec<Vec<int>> initial_prev_feat_idx(this->N_FEATURES);
    for (uint32_t feat_id = 0; feat_id < this->feat_pts.back().size(); ++feat_id) {
        this->cm1_feat_pts_size.at(feat_id) = (this->feat_pts.rbegin() + 1)->at(feat_id).dimension(1);
        initial_prev_feat_idx.at(feat_id) = this->prev_feat_idx.at(feat_id);
    }

    bool first_iteration = true;
    for (int op = 0; op < this->param.opt_iters; op++) {
        if (op > 0) {
            last_transform = ref;
            first_iteration = false;
        }
        this->clearVolatileTracks();

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
            auto &pfeat = this->prev_feature_candidatesT.at(j);

            /// 2. Update average point position from each feature tracks
            this->ave_pts.at(j).resize(3, this->feature_tracks.at(j).size());
//#pragma omp parallel for
            for (uint32_t t_idx = 0; t_idx < this->feature_tracks.at(j).size(); t_idx++) {
                const auto &track = this->feature_tracks.at(j).at(t_idx);
                this->ave_pts.at(j).block<3, 1>(0, t_idx) = track.geometry.block<3, 1>(3, 0).cast<float>();
            }
            /// 3. Build kd trees on previous two scans, and on average track locations
            if (cfeat.cols() > 10) {
                auto cur_kd_idx = Nabo::NNSearchF::createKDTreeLinearHeap(cfeat);
                Eigen::MatrixXi nn_idx;
                Eigen::MatrixXf nn_dist;
                this->cur_feat_idx.at(j).resize(static_cast<unsigned long>(cfeat.size()));
                std::fill(this->cur_feat_idx.at(j).begin(), this->cur_feat_idx.at(j).end(), -1);
                if (!this->feature_tracks.at(j).empty()) {
                    /// 4. Find correspondences for existing feature tracks in current scan
                    const int knn = 6;
                    nn_idx.resize(knn, this->ave_pts.at(j).cols());
                    nn_dist.resize(knn, this->ave_pts.at(j).cols());
                    cur_kd_idx->knn(this->ave_pts.at(j),
                                    nn_idx,
                                    nn_dist,
                                    knn,
                                    0,
                                    Nabo::NNSearchF::SORT_RESULTS,
                                    this->param.max_correspondence_dist);

                    this->extendFeatureTracks(nn_idx, nn_dist, j, first_iteration);
                }
                this->prev_feat_idx.at(j) = initial_prev_feat_idx.at(j);
                if (pfeat.cols() > 10) {
                    const int knn = 5;
                    auto curm1_kd_idx = Nabo::NNSearchF::createKDTreeLinearHeap(pfeat);
                    /// 5. Create new feature tracks between new and old scan
                    nn_idx.resize(knn, cfeat.cols());
                    nn_dist.resize(knn, cfeat.cols());
                    curm1_kd_idx->knn(
                      cfeat, nn_idx, nn_dist, knn, 0, Nabo::NNSearchF::SORT_RESULTS, this->param.max_correspondence_dist);
                    this->createNewFeatureTracks(nn_idx, nn_dist, j, cur_kd_idx, first_iteration);
                    delete curm1_kd_idx;
                }
                delete cur_kd_idx;
            }

            /// 6.5 Transform all features to the start of the window
            for (uint32_t i = 0; i < this->feat_pts.size(); i++) {
                auto &feat = this->feat_pts.at(i).at(j);
                auto &featT = this->feat_pts_T.at(i).at(j);
                this->transformer.transformToStart(feat, featT, i);
            }
        }
        if (!(this->param.only_extract_features)) {
//            auto prev_transform = this->cur_trajectory.back().pose;
            for (uint32_t j = 0; j < this->N_FEATURES; j++) {
                this->setupSkipPoint(j);
            }

            problem.reset(nullptr);

            ceres::Problem::Options options;
            options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            options.disable_all_safety_checks = true;

            problem = std::make_unique<ceres::Problem>(options);

            ceres::Solver::Summary summary;

            if (!this->runOptimization(*problem, summary))
                return false;

//            Vec6 diff = this->cur_trajectory.back().pose.manifoldMinus(prev_transform);
//            prev_transform = this->cur_trajectory.back().pose;
//            std::cout << diff.cwiseAbs().sum();
        }
    }
    //    if(!(this->param.only_extract_features)) {
    //        this->calculateCovariance(*problem);
    //    }

    return true;
}

void LaserOdom::trackResiduals(ceres::Problem &problem, uint32_t f_idx, VecE<FeatureTrack> &track_list) {
    for (auto &track : track_list) {
        if (f_idx == 2) {
            this->local_params.emplace_back(std::make_shared<PlaneParameterization>());
        } else {
            this->local_params.emplace_back(std::make_shared<LineParameterization>());
        }
        problem.AddParameterBlock(track.geometry.data(), 6, this->local_params.back().get());
        if (!(track.optimize)) {
            problem.SetParameterBlockConstant(track.geometry.data());
        }
        for (uint32_t p_idx = 0; p_idx < track.mapping.size(); ++p_idx) {
            const auto &map = track.mapping.at(p_idx);
            this->loss_functions.emplace_back(new BisquareLoss(this->param.robust_param));
            float pt_time =
              this->scan_stampsf.at(map.scan_idx) + this->feat_pts.at(map.scan_idx).at(f_idx)(3, map.pt_idx);
            if (f_idx == 2) {
                this->costs.emplace_back(new PlaneResidual<12, 6, 12, 6>(p_idx,
                                                                         f_idx,
                                                                         pt_time,
                                                                         &track,
                                                                         &(this->feat_pts_T),
                                                                         &(this->ptT_jacobians.at(map.state_id)),
                                                                         &(this->jacobian_stamps.at(map.state_id))));
            } else {
                this->costs.emplace_back(new LineResidual<12, 6, 12, 6>(p_idx,
                                                                        f_idx,
                                                                        pt_time,
                                                                        &track,
                                                                        &(this->feat_pts_T),
                                                                        &(this->ptT_jacobians.at(map.state_id)),
                                                                        &(this->jacobian_stamps.at(map.state_id))));
            }
            uint32_t start_offset = track.mapping.at(p_idx).state_id;
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

void LaserOdom::buildResiduals(ceres::Problem &problem) {
    this->costs.clear();
    this->local_params.clear();
    this->loss_functions.clear();
    for (uint32_t f_idx = 0; f_idx < this->N_FEATURES; ++f_idx) {
        this->trackResiduals(problem, f_idx, this->feature_tracks.at(f_idx));
    }
    for (uint32_t state_id = 0; state_id < this->cur_trajectory.size(); ++state_id) {
        auto &state = this->cur_trajectory[state_id];
        this->local_params.emplace_back(std::make_shared<NullSE3Parameterization>());
        problem.AddParameterBlock(state.pose.storage.data(), 12, this->local_params.back().get());
        problem.AddParameterBlock(state.vel.data(), 6);
        if (state_id > 0) {
            // create constant velocity residuals between each state
            auto &pstate = this->cur_trajectory[state_id - 1];
            auto dT = this->trajectory_stamps.at(state_id) - this->trajectory_stamps.at(state_id - 1);
            Mat12 weight;
            this->cv_model->calculateLinInvCovariance(
              weight, this->trajectory_stamps.at(state_id - 1), this->trajectory_stamps.at(state_id));
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
    // add prior factor on starting velocity
    if (this->prior_twist.sum() != 0.0) {
        //        Mat6 sqrt_info = (this->twist_covar + this->prev_delta_t * this->param.Qc).inverse().sqrt();
        Mat6 sqrt_info = (this->prev_delta_t * this->param.Qc).inverse().sqrt();
        this->costs.emplace_back(new ceres::NormalPrior(sqrt_info, this->prior_twist));
        problem.AddResidualBlock(this->costs.back().get(), nullptr, this->cur_trajectory.front().vel.data());
    }

    // finally, just fix the first pose
    problem.SetParameterBlockConstant(this->cur_trajectory.front().pose.storage.data());
}

void LaserOdom::resetTrajectory() {
    for (auto tra : this->cur_trajectory) {
        tra.pose.setIdentity();
        tra.vel.setZero();
    }
    this->prior_twist.setZero();
}

void LaserOdom::checkTrackValidity() {
    for (uint32_t i = 0; i < this->feature_tracks.size(); ++i) {
        for (uint32_t j = 0; j < this->feature_tracks.at(i).size(); ++j) {
            const auto &track = this->feature_tracks.at(i).at(j);
            for (const auto &map : track.mapping) {
                if (this->feat_pts.at(map.scan_idx).at(i).dimension(1) < map.pt_idx) {
                    throw std::runtime_error("Invalid feature track");
                }
            }
        }
    }
}

}  // namespace wave
