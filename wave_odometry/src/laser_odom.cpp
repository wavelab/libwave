#include <Eigen/Eigen>
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

    size_t n_features = this->feature_extractor.param.feature_definitions.size();

    // todo don't do this
    this->cur_feature_candidates.resize(n_features);
    this->cur_feature_candidatesT.resize(n_features);
    this->prev_feature_candidates.resize(n_features);
    this->prev_feature_candidatesT.resize(n_features);
    this->cur_feature_points.resize(n_features);
    this->cur_feature_pointsT.resize(n_features);
    this->prev_feature_points.resize(n_features);
    this->prev_feature_pointsT.resize(n_features);
    this->ave_pts.resize(n_features);
    this->feature_tracks.resize(n_features);
    this->cur_feat_idx.resize(n_features);
    this->prev_feat_idx.resize(n_features);
    this->undis_tracks.resize(n_features);
    this->undis_features.resize(n_features);
    this->undis_candidates_cur.resize(n_features);
    this->undis_candidates_prev.resize(n_features);
    this->binner.resize(n_features);
    this->cm1_feat_pts_size.resize(n_features);

    this->cur_scan.resize(n_ring);
    this->signals.resize(n_ring);

    for (uint32_t i = 0; i < n_ring; i++) {
        this->cur_scan.at(i) = Eigen::Tensor<float, 2>(5, this->MAX_POINTS);
        this->signals.at(i).emplace(std::make_pair(Signal::RANGE, Eigen::Tensor<float, 1>(this->MAX_POINTS)));
        this->signals.at(i).emplace(std::make_pair(Signal::INTENSITY, Eigen::Tensor<float, 1>(this->MAX_POINTS)));
    }

    this->indices.resize(n_features);
    for (uint32_t i = 0; i < n_features; i++) {
        this->indices.at(i).resize(this->param.n_ring);
        this->binner.at(i).setParams(this->param.binner_params);
    }

    this->range_sensor = std::make_shared<RangeSensor>(param.sensor_params);

    this->key_distance.setZero();
    this->prior_twist.setZero();
}

void LaserOdom::updateParams(const LaserOdomParams &new_params) {
    this->param = new_params;
}

LaserOdomParams LaserOdom::getParams() const {
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
    size_t n_features = this->feature_extractor.param.feature_definitions.size();
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
            pt.intensity = this->signals.at(ring_id).at(Signal::INTENSITY)(i);
            this->undistorted_cld.push_back(pt);
        }
    }
    auto n_scans = this->feat_pts_T.size();
    this->undis_features.resize(n_scans);
    for (uint32_t feat_id = 0; feat_id < n_features; ++feat_id) {
        this->undis_tracks.at(feat_id) = this->feature_tracks.at(feat_id);
        for (uint32_t i = 0; i < n_scans; ++i) {
            this->undis_features.at(i).resize(n_features);
            this->undis_features.at(i).at(feat_id).clear();
            if(i == n_scans - 2) {
                for (uint32_t j = 0; j < this->prev_feature_pointsT.at(feat_id).cols(); ++j) {
                    pcl::PointXYZ pt;
                    pt.x = this->prev_feature_pointsT.at(feat_id)(0, j);
                    pt.y = this->prev_feature_pointsT.at(feat_id)(1, j);
                    pt.z = this->prev_feature_pointsT.at(feat_id)(2, j);
                    this->undis_features.at(i).at(feat_id).push_back(pt);
                }
            } else if (i == n_scans - 1) {
                for (uint32_t j = 0; j < this->cur_feature_pointsT.at(feat_id).cols(); ++j) {
                    pcl::PointXYZ pt;
                    pt.x = this->cur_feature_pointsT.at(feat_id)(0, j);
                    pt.y = this->cur_feature_pointsT.at(feat_id)(1, j);
                    pt.z = this->cur_feature_pointsT.at(feat_id)(2, j);
                    this->undis_features.at(i).at(feat_id).push_back(pt);
                }
            } else {
                for (uint32_t j = 0; j < this->feat_pts_T.at(i).at(feat_id).cols(); ++j) {
                    pcl::PointXYZ pt;
                    pt.x = this->feat_pts_T.at(i).at(feat_id)(0, j);
                    pt.y = this->feat_pts_T.at(i).at(feat_id)(1, j);
                    pt.z = this->feat_pts_T.at(i).at(feat_id)(2, j);
                    this->undis_features.at(i).at(feat_id).push_back(pt);
                }
            }
        }
        this->undis_candidates_prev.at(feat_id).clear();
        this->undis_candidates_cur.at(feat_id).clear();

        const auto &c_feat = this->cur_feature_candidatesT.at(feat_id);
        for (uint32_t i = 0; i < c_feat.cols(); ++i) {
            pcl::PointXYZ pt;
            pt.x = c_feat(0, i);
            pt.y = c_feat(1, i);
            pt.z = c_feat(2, i);
            this->undis_candidates_cur.at(feat_id).push_back(pt);
        }
        const auto &p_feat = this->prev_feature_candidatesT.at(feat_id);
        for (uint32_t i = 0; i < p_feat.cols(); ++i) {
            pcl::PointXYZ pt;
            pt.x = p_feat(0, i);
            pt.y = p_feat(1, i);
            pt.z = p_feat(2, i);
            this->undis_candidates_prev.at(feat_id).push_back(pt);
        }

    }
}

template<class S_TYPE, class D_TYPE>
void LaserOdom::copyTrajectory(const VecE<S_TYPE> &src, VecE<D_TYPE> &dst) {
    dst.resize(src.size());
    for (uint32_t i = 0; i < src.size(); i++) {
        dst.at(i).pose = src.at(i).pose;
        dst.at(i).vel = src.at(i).vel;
    }
}

void LaserOdom::updateFeatureCandidates() {
    size_t n_features = this->feature_extractor.param.feature_definitions.size();
    for (uint32_t i = 0; i < n_features; i++) {
        auto &feat = this->cur_feature_candidates.at(i);
        auto &pfeat = this->prev_feature_candidates.at(i);
        std::swap(feat, pfeat);
        auto &c_idx = this->cur_feat_idx.at(i);
        auto &p_idx = this->prev_feat_idx.at(i);
        std::swap(c_idx, p_idx);

        long feat_count = 0;
        for(uint32_t ring_id = 0; ring_id < this->param.n_ring; ++ring_id) {
            feat_count += this->indices.at(i).at(ring_id).dimension(0);
        }

        feat.resize(5, feat_count);
        c_idx.resize(feat_count);
        std::fill(c_idx.begin(), c_idx.end(), false);

        long counter = 0;
        for(uint32_t ring_id = 0; ring_id < this->param.n_ring; ++ring_id) {
            auto &feat_list = this->indices.at(i).at(ring_id);

            for (uint32_t k = 0; k < feat_list.dimension(0); ++k) {
                const int &idx = feat_list(k);
                feat.slice(ar2({0, counter}), ar2({4, 1})) = this->cur_scan.at(ring_id).slice(ar2({0, idx}), ar2({4, 1}));
                feat(4, counter) = ring_id;
                ++counter;
            }
        }
    }
}

void LaserOdom::addPoints(const std::vector<PointXYZIR> &pts, const int tick, TimeType stamp) {
    if (tick - this->prv_tick < -1000) {  // current scan has ended
        this->feature_extractor.getFeatures(this->cur_scan, this->signals, this->counters, this->indices);
        this->updateFeatureCandidates();
        if (this->initialized) {
            this->match(stamp);
            auto &last = this->cur_trajectory.back().pose;
            auto &second_last = this->cur_trajectory.at(this->cur_trajectory.size() - 2).pose;
            this->key_distance = this->key_distance + last.manifoldMinus(second_last);

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
        //this->checkPosesNormalized();
        this->rollover(stamp);
        //this->checkPosesNormalized();
        std::fill(this->counters.begin(), this->counters.end(), 0);
    }

    for (PointXYZIR pt : pts) {
        if (counters.at(pt.ring) >= static_cast<int>(this->MAX_POINTS)) {
            throw std::out_of_range("Rebuild with higher max points");
        }
        float range = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        auto diff = stamp - this->scan_stamps_chrono.back();
        float secon = std::chrono::duration<float, std::ratio<1>>(diff).count();

        if (secon < 0.0f) {
            secon = 0;
        }
        float azimuth = std::atan2(pt.y, pt.x);
        if (azimuth < 0) {
            azimuth += 2 * M_PI;
        }

        this->cur_scan.at(pt.ring)(0, counters.at(pt.ring)) = pt.x;
        this->cur_scan.at(pt.ring)(1, counters.at(pt.ring)) = pt.y;
        this->cur_scan.at(pt.ring)(2, counters.at(pt.ring)) = pt.z;
        this->cur_scan.at(pt.ring)(3, counters.at(pt.ring)) = secon;
        this->cur_scan.at(pt.ring)(4, counters.at(pt.ring)) = azimuth;

        this->signals.at(pt.ring).at(Signal::RANGE)(counters.at(pt.ring)) = range;
        this->signals.at(pt.ring).at(Signal::INTENSITY)(counters.at(pt.ring)) = pt.intensity;

        this->counters.at(pt.ring)++;
    }

    this->prv_tick = tick;
}

void LaserOdom::fluidToStaticUpdate() {
    for (auto &tracks : this->feature_tracks) {
        for (auto &track : tracks) {
            for (const auto &map : track.fluid_mapping){
                if (map.scan_idx == this->scan_stampsf.size() - 2) {
                    track.static_mapping.emplace_back(map);
                }
            }
            track.fluid_mapping.clear();
        }
    }
}

void LaserOdom::updateTracks() {
    // transform all landmark states to start of current scan and decrement the scan id of each associated feature point
    const auto transform = this->cur_trajectory.at(this->param.num_trajectory_states - 1).pose.transformInverse() *
            this->T_O_L;
    for (auto &tracks : this->feature_tracks) {
        VecE<FeatureTrack> updated_tracks;
        for (auto &track : tracks) {
            double track_range = track.geometry.block<3, 1>(3, 0).norm();
            if (track_range > this->param.track_keep_range) {
                continue;
            }
            if (track.length == 0) {
                track.age = 0;
            }
            Vec<FeatureTrack::Mapping> updated_mapping;
            for (const auto &map : track.static_mapping) {
                if (map.scan_idx != 0) {
                    updated_mapping.emplace_back(map);
                    updated_mapping.back().scan_idx -= 1;
                    updated_mapping.back().state_id -= this->param.num_trajectory_states - 1;
                }
            }

            if (updated_mapping.size() < track.static_mapping.size()) {
//                if (track.length == 0) {
//                    throw std::runtime_error("reducing zero length");
//                }
                --(track.length);
            }
            std::swap(track.static_mapping, updated_mapping);

            track.geometry.block<3, 1>(0, 0) = transform.storage.block<3, 3>(0, 0) * track.geometry.block<3, 1>(0, 0);
            auto ref = track.geometry.block<3, 1>(3, 0);
            transform.transform(ref, ref);

            track.age++;

            updated_tracks.emplace_back(std::move(track));
        }
        std::swap(tracks, updated_tracks);
    }
    T_O_L.setIdentity();
}

void LaserOdom::rollover(TimeType stamp) {
    if (this->param.n_window > 2) {
        this->fluidToStaticUpdate();
    }
    // If there are n_scans worth of data, rotate
    size_t n_features = this->feature_extractor.param.feature_definitions.size();
    if (this->scan_stamps_chrono.size() == this->param.n_window) {
        this->prior_twist = this->cur_trajectory.front().vel;
        this->prev_delta_t = (double) this->scan_stampsf.at(1);
        // Perform a left rotation
        this->updateTracks();
        std::rotate(this->feat_pts.begin(), this->feat_pts.begin() + 1, this->feat_pts.end());
        std::rotate(this->feat_pts_T.begin(), this->feat_pts_T.begin() + 1, this->feat_pts_T.end());
        std::rotate(
                this->scan_stamps_chrono.begin(), this->scan_stamps_chrono.begin() + 1, this->scan_stamps_chrono.end());

        //this->checkPosesNormalized();

        std::rotate(this->cur_trajectory.begin(),
                    this->cur_trajectory.begin() + this->param.num_trajectory_states - 1,
                    this->cur_trajectory.end());

        //this->checkPosesNormalized();

        T_TYPE T_01 = this->cur_trajectory.front().pose.transformInverse();

//        Mat3 R = T_01.storage.block<3,3>(0,0);
//        Mat3 RtR = this->cur_trajectory.front().pose.storage.block<3,3>(0,0) * R;
//        double Rdet = R.determinant();
//        if ((RtR - Mat3::Identity()).norm() > 1e-4 || std::abs(Rdet - 1) > 1e-4) {
//            std::cout << RtR << "\n";
//            std::cout << Rdet << "\n";
//            std::cout << this->cur_trajectory.front().pose.storage << "\n";
//            std::cout << T_01.storage << "\n";
//
//            throw std::runtime_error("Inverse transform not normalized");
//        }

        // adjust timestamps and trajectory states to start of new window
        for (uint32_t i = 1; i < this->cur_trajectory.size(); ++i) {
            this->cur_trajectory.at(i).pose = T_01 * this->cur_trajectory.at(i).pose;
        }
        this->cur_trajectory.front().pose.setIdentity();

        //this->checkPosesNormalized();
        this->scan_stamps_chrono.back() = stamp;

    } else {
        // grow storage
        this->scan_stamps_chrono.emplace_back(stamp);
        this->scan_stampsf.resize(this->scan_stamps_chrono.size());
        VecE<Eigen::Tensor<float, 2>> vec(n_features);
        VecE<MatX> vec2(n_features);

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

    auto n_scans = this->scan_stampsf.size();
    if (n_scans > 2) {
        for (uint32_t feat_id = 0; feat_id < this->prev_feature_points.size(); ++feat_id) {
            this->feat_pts.at(n_scans - 3).at(feat_id) = this->prev_feature_points.at(feat_id);
        }
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
        array.resize(n_features);
    }

    if (!this->initialized) {
        size_t feature_count = 0;
        for (const auto &cands : this->prev_feature_candidates) {
            feature_count += cands.dimension(1);
        }

        if (feature_count >= (size_t) (this->param.min_features)) {
            this->initialized = true;
        }
    }
}

bool LaserOdom::runOptimization(ceres::Problem &problem, ceres::Solver::Summary &summary, int opt_iter) {
    this->T_O_L_added = false;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.use_explicit_schur_complement = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.max_num_iterations = this->param.max_inner_iters;
    options.max_num_consecutive_invalid_steps = 30;
    options.logging_type = ceres::LoggingType::SILENT;
//    options.use_nonmonotonic_steps = true;

    ceres::EvaluationCallback *callback = new OdometryCallback(&(this->feat_pts),
                                                               &(this->prev_feature_points),
                                                               &(this->cur_feature_points),
                                                               &(this->feat_pts_T),
                                                               &(this->prev_feature_pointsT),
                                                               &(this->cur_feature_pointsT),
                                                               &(this->cur_trajectory),
                                                               &(this->ptT_jacobians),
                                                               &(this->jacobian_stamps),
                                                               &(this->trajectory_stamps),
                                                               &(this->scan_stampsf),
                                                               &(this->transformer));
    options.evaluation_callback = callback;

    std::shared_ptr<ceres::ParameterBlockOrdering> param_ordering = std::make_shared<ceres::ParameterBlockOrdering>();
    this->buildResiduals(problem, *param_ordering, opt_iter);

    options.linear_solver_ordering = param_ordering;

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
        std::string print_string = summary.BriefReport();
        print_string = print_string + " " + std::to_string(summary.total_time_in_seconds);
        LOG_INFO("%s", print_string.c_str());
        LOG_INFO("Residual Blocks: %d", problem.NumResidualBlocks());
    }

    return true;
}

void LaserOdom::calculateCovariance(ceres::Problem &problem) {
    ceres::Covariance::Options covar_options;
    covar_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
    covar_options.algorithm_type = ceres::CovarianceAlgorithmType::SPARSE_QR;
    covar_options.null_space_rank = -1;
    // If the loss function is applied, residuals with large error result in rank-deficient Jacobian
    covar_options.apply_loss_function = false;

    if (this->param.solver_threads < 1) {
        covar_options.num_threads = std::thread::hardware_concurrency();
    } else {
        covar_options.num_threads = this->param.solver_threads;
    }

    ceres::Covariance covariance(covar_options);
    std::vector<const double *> blocks;
    blocks.emplace_back(this->cur_trajectory.front().vel.data());
    if (!covariance.Compute(blocks, &problem)) {
        this->covar_age++;
    } else {
        this->covar_age = 0;
        covariance.GetCovarianceMatrix(blocks, this->twist_covar.data());
    }
}

template<typename Derived, typename Derived1, typename Derived2>
bool LaserOdom::findLineCorrespondences(std::vector<uint32_t> &matches,
                                        std::vector<bool> &used_points,
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
        if (used_points.at(index(i))) {
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

template<typename Derived, typename Derived1, typename Derived2>
bool LaserOdom::findPlaneCorrespondences(std::vector<uint32_t> &matches,
                                         Vec<bool> &used_points,
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
        if (used_points.at(index(i))) {
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
            // calculate cos(theta) between the vectors and
            if (std::abs((AmB.transpose() * AmO)(0)) > 0.95) {
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
    if (matches.size() >= 3) {
        return true;
    }
    return false;
}

void LaserOdom::findSpecificCorrespondences(Nabo::NNSearchF* kd_tree, const MatXf &query, const Eigen::Tensor<float, 2> &queryt, Eigen::Tensor<float, 2> &feat_pts, Vec<bool> &used_idx, int scan_id, int feat_id) {
    const auto &feat_def = this->feature_extractor.param.feature_definitions.at(feat_id);
    bool line_residual = feat_def.criteria.front().sel_pol != NEAR_ZERO;
    auto &current_tracks = this->feature_tracks.at(feat_id);

    auto T_EO = this->cur_trajectory.back().pose.transformInverse();

    int knn = kd_tree->cloud.cols() < this->param.knn ? kd_tree->cloud.cols() : this->param.knn;
    MatXf dist(knn, query.cols());
    Eigen::MatrixXi idx(knn, query.cols());

    if (query.cols() > 0) {
        kd_tree->knn(query,
                            idx,
                            dist,
                     knn,
                            0,
                            Nabo::NNSearchF::ALLOW_SELF_MATCH | Nabo::NNSearchF::SORT_RESULTS,
                            this->param.max_correspondence_dist);
    }

    Eigen::Tensor<float, 2> new_feat_points(4, query.cols());
    long new_feat_cnt = 0;

    Vec<Vec<std::pair<long, double>>> potential_matches(current_tracks.size());

    for (uint32_t j = 0; j < query.cols(); ++j) {
        Vec3f pt = query.block<3, 1>(0, j);

        /// Evaluate residual errors for any points found within max correspondence distance
        Vec<std::pair<int, double>> errors;
        for (uint32_t i = 0; i < idx.rows(); ++i) {
            if (std::isinf(dist(i, j))) {
                break;
            }
            auto &track = current_tracks.at(idx(i, j));

            if (line_residual) {
                /// Special logic for line residuals because an edge should only cross a laser ring in one place
                double error = calculateLineError(track.geometry, pt);
                if (error < this->param.max_linear_residual_val) {
                    if (errors.empty()) {
                        errors.emplace_back(std::make_pair(idx(i, j), error));
                    } else if (std::get<1>(errors.front()) > error) {
                        errors.front() = std::make_pair(idx(i, j), error);
                    }
                }
            } else {
                double error = calculatePlaneError(track.geometry, pt);
                if (error < this->param.max_planar_residual_val) {
                    errors.emplace_back(std::make_pair(idx(i, j), error));
                }
            }
        }
        if (errors.empty()) {
            continue;
        }
        std::sort(errors.begin(), errors.end(),
                  [](const std::pair<int, double> &lhs, const std::pair<int, double> &rhs) {
                      return std::get<1>(lhs) < std::get<1>(rhs);
                  });
        auto t_id = std::get<0>(errors.front());

        potential_matches.at(t_id).emplace_back(std::make_pair(j, std::get<1>(errors.front())));
    }

    for (uint32_t t_id = 0; t_id < potential_matches.size(); ++t_id) {
        auto& c_match = potential_matches.at(t_id);
        if (c_match.empty()) {
            continue;
        }

        std::sort(c_match.begin(), c_match.end(),
                  [](const std::pair<long, double> &lhs, const std::pair<long, double> &rhs) {
                      return std::get<1>(lhs) < std::get<1>(rhs);
                  });

        auto &track = this->feature_tracks.at(feat_id).at(t_id);

        for (uint32_t p_id = 0; p_id < this->param.knn && p_id < c_match.size(); ++p_id) {
            FeatureTrack::Mapping new_map;
            const auto pt_id = std::get<0>(c_match.at(p_id));
            float pt_time = this->scan_stampsf.at(scan_id) + queryt(3, pt_id);
            auto bnd =
                    std::upper_bound(this->trajectory_stamps.begin(), this->trajectory_stamps.end(), pt_time);
            auto traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);

            new_map.scan_idx = scan_id;
            new_map.state_id = traj_idx;

            if (scan_id == this->scan_stampsf.size() - 1) {
                Vec6 geometry_at_end;
                geometry_at_end.block<3,1>(0,0) = T_EO.storage.block<3,3>(0,0) * track.geometry.block<3,1>(0,0);
                geometry_at_end.block<3,1>(3,0) = T_EO.transform(track.geometry.block<3,1>(3,0));
                this->binner.at(feat_id).bin(geometry_at_end);
            }

            used_idx.at(pt_id) = true;
            new_map.pt_idx = new_feat_cnt;

            Eigen::array<int, 2> offsets_new = {0, static_cast<int>(new_feat_cnt)};
            Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(pt_id)};
            Eigen::array<int, 2> extents = {4, 1};

            new_feat_points.slice(offsets_new, extents) = queryt.slice(offsets_candidate, extents);
            ++new_feat_cnt;

            track.fluid_mapping.emplace_back(new_map);
            track.length = uniqueElements(track);
        }
    }

    feat_pts = new_feat_points.slice(ar2{0,0}, ar2{4, new_feat_cnt});
}

void LaserOdom::findCorrespondences(uint32_t feat_id) {
    /// Update average point position from each feature tracks
    this->ave_pts.at(feat_id).resize(3, this->feature_tracks.at(feat_id).size());
    for (uint32_t t_idx = 0; t_idx < this->feature_tracks.at(feat_id).size(); t_idx++) {
        const auto &track = this->feature_tracks.at(feat_id).at(t_idx);
        this->ave_pts.at(feat_id).block<3, 1>(0, t_idx) = track.geometry.block<3, 1>(3, 0).cast<float>();
    }
    if (this->ave_pts.at(feat_id).cols() == 0) {
        return;
    }
    /// Transform all candidate points to the start of the window
    this->transformer.transformToStart(this->cur_feature_candidates.at(feat_id),
                                       this->cur_feature_candidatesT.at(feat_id),
                                       this->scan_stamps_chrono.size() - 1);
    this->transformer.transformToStart(this->prev_feature_candidates.at(feat_id),
                                       this->prev_feature_candidatesT.at(feat_id),
                                       this->scan_stamps_chrono.size() - 2);

    auto kd_tree = Nabo::NNSearchF::createKDTreeLinearHeap(this->ave_pts.at(feat_id), 3);

    this->findSpecificCorrespondences(kd_tree, this->cur_feature_candidatesT.at(feat_id),
            this->cur_feature_candidates.at(feat_id), this->cur_feature_points.at(feat_id),
            this->cur_feat_idx.at(feat_id),
            this->scan_stampsf.size() - 1, feat_id);
    this->findSpecificCorrespondences(kd_tree, this->prev_feature_candidatesT.at(feat_id),
            this->prev_feature_candidates.at(feat_id), this->prev_feature_points.at(feat_id),
            this->prev_feat_idx.at(feat_id),
            this->scan_stampsf.size() - 2, feat_id);

    delete kd_tree;
}

void LaserOdom::createNewGeometry(const MatXf &points,
                                  const Vec<uint32_t> &indices,
                                  ResidualType residual_type,
                                  Vec6 &geometry) {
    if (residual_type == PointToLine) {
        geometry.block<3, 1>(3, 0) =
                (points.block<3, 1>(0, indices.at(0)).cast<double>() +
                 points.block<3, 1>(0, indices.at(1)).cast<double>()) /
                2.0;

        geometry.block<3, 1>(0, 0) =
                points.block<3, 1>(0, indices.at(0)).cast<double>() -
                points.block<3, 1>(0, indices.at(1)).cast<double>();
    } else {
        geometry.block<3, 1>(3, 0) =
                (points.block<3, 1>(0, indices.at(0)).cast<double>() +
                 points.block<3, 1>(0, indices.at(1)).cast<double>() +
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
void LaserOdom::createNewFeatureTracks(uint32_t feat_id, const VecE<FeatureTrack> &tracks) {
    // For any successes, add to feature tracks
    // Tracks themselves are left empty, correspondences are found before each optimization iteration
    auto T_EO = this->cur_trajectory.back().pose.transformInverse();

    for (const auto &candidate_track : tracks) {
        Vec6 geometry_at_end;
        geometry_at_end.block<3,1>(0,0) = T_EO.storage.block<3,3>(0,0) * candidate_track.geometry.block<3,1>(0,0);
        geometry_at_end.block<3,1>(3,0) = T_EO.transform(candidate_track.geometry.block<3,1>(3,0));
        if (this->binner.at(feat_id).bin(geometry_at_end)) {
            this->feature_tracks.at(feat_id).emplace_back(candidate_track);
            auto &track_ref = this->feature_tracks.at(feat_id).back();
            track_ref.static_mapping.clear();
            track_ref.fluid_mapping.clear();
            track_ref.age = 0;
            track_ref.length = 2;
        }
    }
}

void LaserOdom::createNewPlanes(Nabo::NNSearchF *tree, const MatXf &dataset, const Eigen::Tensor<float, 2> &datasetT,
                                const MatXf &query, const Vec<bool> &points_used, VecE <FeatureTrack> &output_tracks,
                                const uint32_t &feat_id) {
    using M_TYPE = std::tuple<uint32_t, int, float>;
    auto T_EO = this->cur_trajectory.back().pose.transformInverse();

    int knn = this->param.knn > dataset.cols() ? dataset.cols() : this->param.knn;
    Eigen::MatrixXi idx(knn, query.cols());
    MatXf dist(knn, query.cols());

    tree->knn(query, idx, dist, knn, 0, Nabo::NNSearchF::ALLOW_SELF_MATCH | Nabo::NNSearchF::SORT_RESULTS,
                          this->param.max_correspondence_dist);

    for (uint32_t j = 0; j < dist.cols(); ++j) {
        Vec<M_TYPE> matches;
        for (int i = 0; i < knn; ++i) {
            if (std::isinf(dist(i,j))) {
                break;
            }
            //skip over any points that already have a correspondence
            if (points_used.at(idx(i,j))) {
                continue;
            }
            matches.emplace_back((uint32_t)(datasetT(4, idx(i,j))), idx(i,j), dist(i,j));
        }
        /// Now need to limit the number of each points from each ring to the 2 best examples
        {
            Vec<M_TYPE> minimal_matches;
            Vec<int> ring_counter(this->param.n_ring);
            std::fill(ring_counter.begin(), ring_counter.end(), 0);
            for (const auto &match : matches) {
                auto r_id = std::get<0>(match);
                if (ring_counter.at(r_id) < 2) {
                    ring_counter.at(r_id)++;
                    minimal_matches.emplace_back(match);
                }
            }
            std::swap(matches, minimal_matches);
        }
        if (matches.size() < 3) {
            continue;
        }
        Vec<uint32_t> pt_idx;
        FeatureTrack new_track;
        for (uint32_t k = 0; k < 3; ++k) {
            auto c_match = matches.at(k);
            pt_idx.emplace_back(std::get<1>(c_match));
        }
        this->createNewGeometry(dataset, pt_idx, PointToPlane, new_track.geometry);
        Vec6 geometry_at_end;
        geometry_at_end.block<3,1>(0,0) = T_EO.storage.block<3,3>(0,0) * new_track.geometry.block<3,1>(0,0);
        geometry_at_end.block<3,1>(3,0) = T_EO.transform(new_track.geometry.block<3,1>(3,0));

        if (this->binner.at(feat_id).spaceInBin(geometry_at_end)) {
            output_tracks.emplace_back(new_track);
        }
    }
}

void LaserOdom::createNewPlaneFeatureTrackCandidates(uint32_t feat_id,
                                                VecE<FeatureTrack> &candidate_tracks) {
    //create kd trees on prev and cur feature points
    Nabo::NNSearchF *cur_kd, *prev_kd;

    if (this->cur_feature_candidatesT.at(feat_id).cols() == 0) {
        cur_kd = nullptr;
    } else {
        cur_kd = Nabo::NNSearchF::createKDTreeLinearHeap(this->cur_feature_candidatesT.at(feat_id));
    }
    if (this->prev_feature_candidatesT.at(feat_id).cols() == 0) {
        prev_kd = nullptr;
    } else {
        prev_kd = Nabo::NNSearchF::createKDTreeLinearHeap(this->prev_feature_candidatesT.at(feat_id));
    }

    VecE<FeatureTrack> initial_prev_tracks, initial_cur_tracks;
    this->createNewPlanes(prev_kd, this->prev_feature_candidatesT.at(feat_id),
                          this->prev_feature_candidates.at(feat_id),
                          this->cur_feature_candidatesT.at(feat_id),
                          this->prev_feat_idx.at(feat_id),
                          initial_prev_tracks,
                          feat_id);
    this->createNewPlanes(cur_kd, this->cur_feature_candidatesT.at(feat_id),
                          this->cur_feature_candidates.at(feat_id),
                          this->prev_feature_candidatesT.at(feat_id),
                          this->cur_feat_idx.at(feat_id),
                          initial_cur_tracks,
                          feat_id);

    this->mergeFeatureTracks(initial_prev_tracks, feat_id);
    this->mergeFeatureTracks(initial_cur_tracks, feat_id);
    /// now have two sets of fresh tracks, can try to find corresponding features between the two sets
    /// going to try to search prev tracks with cur tracks as a query
    for (const auto &track : initial_prev_tracks) {
        candidate_tracks.emplace_back(track);
    }
    for (const auto &track : initial_cur_tracks) {
        candidate_tracks.emplace_back(track);
    }
    MatXf dataset(3, initial_prev_tracks.size());
    for (uint32_t col = 0; col < initial_prev_tracks.size(); ++col) {
        dataset.block<3,1>(0,col) = initial_prev_tracks.at(col).geometry.block<3,1>(3,0).cast<float>();
    }
    MatXf query(3, initial_cur_tracks.size());
    for (uint32_t col = 0; col < initial_cur_tracks.size(); ++col) {
        query.block<3,1>(0,col) = initial_cur_tracks.at(col).geometry.block<3,1>(3,0).cast<float>();
    }
    if (dataset.cols() == 0) {
        return;
    }
    auto kd_tree = Nabo::NNSearchF::createKDTreeLinearHeap(dataset);

    long knn = this->param.knn > dataset.cols() ? dataset.cols() : this->param.knn;
    MatXf dist(knn, initial_cur_tracks.size());
    Eigen::MatrixXi idx(knn, initial_cur_tracks.size());

    kd_tree->knn(query, idx, dist, knn, 0, Nabo::NNSearchF::ALLOW_SELF_MATCH | Nabo::NNSearchF::SORT_RESULTS);

    for (uint32_t j = 0; j < idx.cols(); ++j) {
        const auto &cur_track = initial_cur_tracks.at(j);
        Vec<std::tuple<int, float>> matches;
        for (uint32_t i = 0; i < idx.rows(); ++i) {
            const auto &prev_track = initial_prev_tracks.at(idx(i,j));
            float dist_cost, dir_cost;
            this->calculatePlaneSimilarity(cur_track.geometry, prev_track.geometry, dist_cost, dir_cost);
            if (dist_cost < this->param.init_planar_dist_threshold && dir_cost < this->param.init_planar_ang_threshold) {
                float score = dist_cost + this->param.ang_scaling_param * dir_cost;
                matches.emplace_back(idx(i,j), score);
            }
        }
        if (!matches.empty()) {
            std::sort(matches.begin(), matches.end(), [](const std::tuple<int, float> &lhs, const std::tuple<int, float> &rhs) {
                return std::get<1>(lhs) < std::get<1>(rhs);
            });
            const auto &prev_track = initial_prev_tracks.at(std::get<0>(matches.front()));
            candidate_tracks.emplace_back(prev_track);
        }
    }
}

void LaserOdom::createNewLineFeatureTrackCandidates(uint32_t feat_id,
                                                    VecE<FeatureTrack> &candidate_tracks) {
    auto c_cols = this->cur_feature_candidatesT.at(feat_id).cols();
    auto p_cols = this->prev_feature_candidatesT.at(feat_id).cols();
    auto cols = c_cols + p_cols;
    MatXf line_candidates(3, cols);

    line_candidates.block(0,0,3,c_cols) = this->cur_feature_candidatesT.at(feat_id);
    line_candidates.block(0,c_cols, 3, p_cols) = this->prev_feature_candidatesT.at(feat_id);
    LineFitter line_fitter(this->param.max_correspondence_dist * 3, 10, this->param.max_init_linear_residual, 6);
    line_fitter.fitLines(line_candidates);
    candidate_tracks = line_fitter.getTracks();

    // merge shortlist and sort by number of measurements
    this->mergeFeatureTracks(candidate_tracks, feat_id);
//    delete kd_tree;
}

void LaserOdom::clearVolatileTracks() {
    for (auto &tracks : this->feature_tracks) {
        for (auto &track : tracks) {
            track.fluid_mapping.clear();
            track.length = uniqueElements(track);
        }
    }
    for (uint32_t feat_id = 0; feat_id < this->feat_pts.back().size(); ++feat_id) {
        this->cur_feature_points.at(feat_id).resize(Eigen::NoChange, 0);
        this->prev_feature_points.at(feat_id).resize(Eigen::NoChange, 0);
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

template<class Derived, class OtherDerived>
double LaserOdom::calculateLineError(const Eigen::MatrixBase<Derived> &geo,
                                     const Eigen::MatrixBase<OtherDerived> &pt) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 6)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<OtherDerived>, 3)

    Vec3 diff = pt.template cast<double>() - geo.template block<3,1>(3,0).template cast<double>();
    double dp = (geo.template block<3,1>(0,0).transpose() * diff)(0);
    Vec3 error3 = diff - dp * geo.template block<3,1>(0,0);

    return error3.norm();
}

template<class Derived, class OtherDerived>
double LaserOdom::calculatePlaneError(const Eigen::MatrixBase<Derived> &geo,
                                     const Eigen::MatrixBase<OtherDerived> &pt) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 6)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<OtherDerived>, 3)

    Vec3 diff = pt.template cast<double>() - geo.template block<3,1>(3,0).template cast<double>();
    double dp = (geo.template block<3,1>(0,0).transpose() * diff)(0);

    return std::abs(dp);
}

void LaserOdom::mergeFeatureTracks(VecE<FeatureTrack> &tracks, uint32_t feat_id) {
    if (tracks.size() <= 1) {
        return;
    }
    const auto& feat_def = this->feature_extractor.param.feature_definitions.at(feat_id);
    MatXf trk_pts;
    trk_pts.resize(3, tracks.size());

    const int knn = this->param.knn < (tracks.size() - 1) ? this->param.knn : (tracks.size() - 1);

    Vec<std::list<uint32_t>> merge_history(tracks.size());
    Vec<uint32_t> addr(tracks.size());
    std::iota(addr.begin(), addr.end(), 0);

    //#pragma omp parallel for
    for (uint32_t t_idx = 0; t_idx < tracks.size(); t_idx++) {
        merge_history.at(t_idx).emplace_back(t_idx);
        const auto &track = tracks.at(t_idx);
        trk_pts.block<3, 1>(0, t_idx) = track.geometry.block<3, 1>(3, 0).cast<float>();
    }

    Nabo::NNSearchF *kd_idx = Nabo::NNSearchF::createKDTreeLinearHeap(trk_pts, 3);

    MatXf dists(knn, trk_pts.cols());
    Eigen::MatrixXi nn_idx(knn, trk_pts.cols());

    kd_idx->knn(trk_pts, nn_idx, dists, knn, 0, 0, this->param.max_correspondence_dist);

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
            if (feat_def.criteria.front().sel_pol == NEAR_ZERO) {
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

            for (const auto &map : merged_track.static_mapping) {
                query_track.static_mapping.emplace_back(map);
            }
            for (const auto &map : merged_track.fluid_mapping) {
                query_track.fluid_mapping.emplace_back(map);
            }
            std::sort(query_track.static_mapping.begin(),
                      query_track.static_mapping.end(),
                      [](const FeatureTrack::Mapping &lhs, const FeatureTrack::Mapping &rhs) {
                          return lhs.scan_idx < rhs.scan_idx;
                      });
            std::sort(query_track.fluid_mapping.begin(),
                      query_track.fluid_mapping.end(),
                      [](const FeatureTrack::Mapping &lhs, const FeatureTrack::Mapping &rhs) {
                          return lhs.scan_idx < rhs.scan_idx;
                      });
            query_track.length = uniqueElements(query_track);
            query_track.age = 0;

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
    for (auto &track : tracks) {
        std::sort(track.static_mapping.begin(), track.static_mapping.end(), [](const FeatureTrack::Mapping &lhs, const FeatureTrack::Mapping &rhs) {
            if (lhs.scan_idx == rhs.scan_idx) {
                return lhs.pt_idx < rhs.pt_idx;
            } else {
                return lhs.scan_idx < rhs.scan_idx;
            }
        });
        std::sort(track.fluid_mapping.begin(), track.fluid_mapping.end(), [](const FeatureTrack::Mapping &lhs, const FeatureTrack::Mapping &rhs) {
            if (lhs.scan_idx == rhs.scan_idx) {
                return lhs.pt_idx < rhs.pt_idx;
            } else {
                return lhs.scan_idx < rhs.scan_idx;
            }
        });
    }
    delete kd_idx;
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
        if (idx == 1) {
            this->cur_trajectory.at(idx).vel = this->cur_trajectory.at(idx - 1).vel;
        } else {
            Vec6 acc = this->cur_trajectory.at(idx - 1).vel - this->cur_trajectory.at(idx - 2).vel;
            this->cur_trajectory.at(idx).vel = this->cur_trajectory.at(idx - 1).vel + acc;
        }
    }
    // as to limit floating point error
    this->trajectory_stamps.back() =
            std::chrono::duration<float, std::ratio<1>>(stamp - this->scan_stamps_chrono.front()).count();

    // Now previous trajectory will hold the "motion generated" trajectory
    this->copyTrajectory(this->cur_trajectory, this->prev_trajectory);
}

void LaserOdom::pointToTracks(const uint32_t feat_id) {
    const auto &feat_def = this->feature_extractor.param.feature_definitions.at(feat_id);

    std::fill(this->cur_feat_idx.at(feat_id).begin(), this->cur_feat_idx.at(feat_id).end(), false);
    std::fill(this->prev_feat_idx.at(feat_id).begin(), this->prev_feat_idx.at(feat_id).end(), false);
    this->binner.at(feat_id).clear();

    if (!this->feature_tracks.at(feat_id).empty()) {
        this->findCorrespondences(feat_id);
    }

    VecE<FeatureTrack> candidate_tracks;
    if (feat_def.criteria.front().sel_pol == NEAR_ZERO) {
        this->createNewPlaneFeatureTrackCandidates(feat_id, candidate_tracks);
    } else {
        this->createNewLineFeatureTrackCandidates(feat_id, candidate_tracks);
    }

    /// Create new feature tracks between new and old scan
    this->createNewFeatureTracks(feat_id, candidate_tracks);
}

void LaserOdom::extendSceneModel(const uint32_t &feat_id) {
    /// Transform all candidate points to the start of the window
    this->transformer.transformToStart(this->cur_feature_candidates.at(feat_id),
                                       this->cur_feature_candidatesT.at(feat_id),
                                       this->scan_stamps_chrono.size() - 1);
    this->transformer.transformToStart(this->prev_feature_candidates.at(feat_id),
                                       this->prev_feature_candidatesT.at(feat_id),
                                       this->scan_stamps_chrono.size() - 2);

    /// try to extend model to set of points that do not correspond to existing model
    this->pointToTracks(feat_id);
}

void LaserOdom::performModelMaintenance(const uint32_t &feat_id) {
    //calculate error statistics & remove tracks that don't fit well.
    const auto &tracks = this->feature_tracks.at(feat_id);
    bool line = this->feature_extractor.param.feature_definitions.at(feat_id).criteria.front().sel_pol != NEAR_ZERO;
    Vec<double> mean_error(tracks.size());
    Vec<double> error_variance(tracks.size());
    for (uint32_t t_id = 0; t_id < tracks.size(); ++t_id) {
        mean_error.at(t_id) = 0;
        Vec<double> errors;
        for (const auto &map : tracks.at(t_id).static_mapping) {
            Vec3 pt = this->feat_pts_T.at(map.scan_idx).at(feat_id).block<3,1>(0,map.pt_idx);
            if (line) {
                errors.emplace_back(calculateLineError(tracks.at(t_id).geometry, pt));
            } else {
                errors.emplace_back(calculatePlaneError(tracks.at(t_id).geometry, pt));
            }
            mean_error.at(t_id) += errors.back();
        }
        for (const auto &map : tracks.at(t_id).fluid_mapping) {
            Vec3 pt;
            if (map.scan_idx == this->feat_pts_T.size() - 1) {
                pt = this->cur_feature_pointsT.at(feat_id).block<3,1>(0, map.pt_idx);
            } else {
                pt = this->prev_feature_pointsT.at(feat_id).block<3,1>(0, map.pt_idx);
            }
            if (line) {
                errors.emplace_back(calculateLineError(tracks.at(t_id).geometry, pt));
            } else {
                errors.emplace_back(calculatePlaneError(tracks.at(t_id).geometry, pt));
            }
            mean_error.at(t_id) += errors.back();
        }
        mean_error.at(t_id) /= static_cast<double>(errors.size());
        error_variance.at(t_id) = 0;
        for (const auto& err : errors) {
            double diff = err - mean_error.at(t_id);
            error_variance.at(t_id) += diff * diff;
        }
        if (errors.size() == 1) {
            error_variance.at(t_id) = 0;
        } else {
            error_variance.at(t_id) /= static_cast<double>(errors.size() - 1);
        }
    }
    VecE<FeatureTrack> good_tracks;
    for (uint32_t t_id = 0; t_id < tracks.size(); ++t_id) {
        if (line && error_variance.at(t_id) > 9e-3) {
            continue;
        } else if (error_variance.at(t_id) > 9e-3) {
            continue;
        }
        good_tracks.emplace_back(tracks.at(t_id));
    }
    std::swap(good_tracks, this->feature_tracks.at(feat_id));
//    matplotlibcpp::plot(mean_error, error_variance, ".");
//    matplotlibcpp::show(true);
}

bool LaserOdom::match(const TimeType &stamp) {
    size_t n_features = this->feature_extractor.param.feature_definitions.size();
    //this->checkPosesNormalized();
    // on the first match, initialization is poor
    static int initial_matches = 10;
    if (initial_matches == 10) {
        for (uint32_t t_id = 0; t_id < this->cur_trajectory.size(); ++t_id) {
            if (t_id == 0) {
                this->cur_trajectory.at(t_id).pose.setIdentity();
            } else {
                this->cur_trajectory.at(t_id).pose = this->cur_trajectory.at(t_id - 1).pose;
                this->cur_trajectory.at(t_id).pose.manifoldPlus(this->cur_trajectory.at(t_id - 1).vel * 0.1);
            }
            this->cur_trajectory.at(t_id).vel = this->param.initial_velocity;
        }
    }
    this->prepTrajectory(stamp);

    std::unique_ptr<ceres::Problem> problem;

    int limit = this->param.opt_iters;
    if (initial_matches == 10) {
        limit *= 10;
    } else if (this->scan_stampsf.size() == this->param.n_window) {
        limit = 1;
    }

    this->transformer.update(this->cur_trajectory, this->trajectory_stamps);

    for (int op = 0; op < limit; op++) {
        if (initial_matches == 10) {
            for (uint32_t j = 0; j < n_features; ++j) {
                this->feature_tracks.at(j).clear();
            }
        }
        if (initial_matches > 0 || this->key_distance.block<3,1>(3,0).norm() > this->param.key_translation) {
            for (uint32_t j = 0; j < n_features; j++) {
                this->extendSceneModel(j);
            }
        }
        if (!(this->param.only_extract_features)) {
            for(int op2 = 0; op2 < 10; ++op2) {
                this->clearVolatileTracks();
                for (uint32_t j = 0; j < n_features; j++) {
                    this->mergeFeatureTracks(this->feature_tracks.at(j), j);
                    this->findCorrespondences(j);
                }

                problem.reset(nullptr);

                ceres::Problem::Options options;
                options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
                options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
                options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
                options.disable_all_safety_checks = true;

                problem = std::make_unique<ceres::Problem>(options);

                ceres::Solver::Summary summary;

                //this->checkPosesNormalized();

                if (!this->runOptimization(*problem, summary, 1))
                    return false;

                //this->checkPosesNormalized();

                if ((summary.iterations.size() == 1) ||
                    ((summary.initial_cost - summary.final_cost) / summary.initial_cost < 1e-7)) {
                    break;
                }
                for (uint32_t j = 0; j < n_features; j++) {
                    this->performModelMaintenance(j);
                }
            }
        }
    }
    if (this->key_distance.block<3,1>(3,0).norm() > this->param.key_translation) {
        this->key_distance.setZero();
    }
    if (initial_matches > 0) {
        --initial_matches;
    }
    if (!(this->param.only_extract_features)) {
//        this->calculateCovariance(*problem);
        problem.reset(nullptr);
    }

    return true;
}

namespace {

void prepareJacobianPointers(const Vec<VecE<MatX>> *jacs,
                             const Vec<float> *jac_stamps,
                             const float pt_time,
                             float &w1,
                             float &w2,
                             VecE<const MatX *> &jacsw1,
                             VecE<const MatX *> &jacsw2) {
    auto iter = std::upper_bound(jac_stamps->begin(), jac_stamps->end(), pt_time);

    float T1, T2;
    if (iter == jac_stamps->end()) {
        T2 = *(--iter);
        T1 = *(--iter);
    } else {
        T2 = *iter;
        T1 = *(--iter);
    }

    w1 = (T2 - pt_time) / (T2 - T1);
    w2 = 1.f - w1;

    auto jac_index = static_cast<uint32_t>(iter - jac_stamps->begin());

    for (uint32_t i = 0; i < 4; ++i) {
        jacsw1.emplace_back(&(jacs->at(i).at(jac_index)));
        jacsw2.emplace_back(&(jacs->at(i).at(jac_index + 1)));
    }
}
}

void LaserOdom::buildResidualsFromMap(ceres::Problem &problem, const Vec <wave::FeatureTrack::Mapping> &mapping,
                                      FeatureTrack &track, uint32_t f_idx, bool use_fixed) {
    const auto& feat_def = this->feature_extractor.param.feature_definitions.at(f_idx);
    for (const auto &map : mapping) {
        const Eigen::Tensor<float, 2>* points;
        const MatX* pointsT;
        // bit of spaghetti
        if(map.scan_idx == this->scan_stampsf.size() - 1) {
            points =  &(this->cur_feature_points.at(f_idx));
            pointsT = &(this->cur_feature_pointsT.at(f_idx));
        } else if (map.scan_idx == this->scan_stampsf.size() - 2) {
            points =  &(this->prev_feature_points.at(f_idx));
            pointsT = &(this->prev_feature_pointsT.at(f_idx));
        } else {
            points =  &(this->feat_pts.at(map.scan_idx).at(f_idx));
            pointsT = &(this->feat_pts_T.at(map.scan_idx).at(f_idx));
        }

        float pt_time =
                this->scan_stampsf.at(map.scan_idx) + (*points)(3, map.pt_idx);

        if (map.pt_idx >= pointsT->cols()) {
            throw std::runtime_error("Point index is out of range inside buildResidualsFromMap");
        }

        Eigen::Map<const Vec3> point(pointsT->data() + 3 * map.pt_idx);
        if (!(point.array().isFinite().maxCoeff())) {
            throw std::runtime_error("Point not finite within residual building");
        }

        const double *cur_point = pointsT->data() + 3 * map.pt_idx;
        float w1, w2;
        VecE<const MatX *> jacsw1, jacsw2;
        prepareJacobianPointers(&(this->ptT_jacobians.at(map.state_id)),
                                &(this->jacobian_stamps.at(map.state_id)),
                                pt_time,
                                w1,
                                w2,
                                jacsw1,
                                jacsw2);

        Mat3 covar_p = Mat3::Identity();
        MatX del_e_del_p, weight;
        if (this->param.use_weighting) {
            this->range_sensor->getEuclideanCovariance(pointsT->data() + 3 * map.pt_idx, covar_p);
        }
        if (feat_def.criteria.front().sel_pol == NEAR_ZERO) {
            if (use_fixed) {
                auto residual = new FixedPlaneResidual<double, 12, 6, 12, 6>(cur_point, jacsw1, jacsw2, w1, w2, track.geometry);
                this->costs.emplace_back(residual);
            } else {
                auto residual = new PlaneResidual<double, 12, 6, 12, 6>(cur_point, jacsw1, jacsw2, w1, w2);
                this->costs.emplace_back(residual);
            }
        } else {
            if (use_fixed) {
                auto residual = new FixedLineResidual<double, 12, 6, 12, 6>(cur_point, jacsw1, jacsw2, w1, w2, track.geometry);
                this->costs.emplace_back(residual);
            } else {
                auto residual = new LineResidual<double, 12, 6, 12, 6>(cur_point, jacsw1, jacsw2, w1, w2);
                this->costs.emplace_back(residual);
            }
        }
        uint32_t start_offset = map.state_id;

        this->loss_functions.emplace_back(new BisquareLoss(this->param.robust_param));

        if (use_fixed) {
            problem.AddResidualBlock(this->costs.back().get(),
                                     this->loss_functions.back().get(),
                                     this->T_O_L.storage.data(),
                                     this->cur_trajectory.at(start_offset).pose.storage.data(),
                                     this->cur_trajectory.at(start_offset).vel.data(),
                                     this->cur_trajectory.at(start_offset + 1).pose.storage.data(),
                                     this->cur_trajectory.at(start_offset + 1).vel.data());
        } else {
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

void LaserOdom::trackResiduals(ceres::Problem &problem, ceres::ParameterBlockOrdering &param_ordering, uint32_t f_idx,
                               VecE <FeatureTrack> &track_list, int opt_iter) {
    const auto& feat_def = this->feature_extractor.param.feature_definitions.at(f_idx);
    for (auto &track : track_list) {
        if (track.static_mapping.empty() && track.fluid_mapping.empty()) {
            continue;
        }
        if (track.geometry(2) < 0) {
            track.geometry.block<3,1>(0,0) = -track.geometry.block<3,1>(0,0);
        }

        //use T_OL to decouple old landmarks from motion estimation
        bool use_fixed = false;
        if (track.age > this->param.n_window) {
            if (!this->T_O_L_added) {
                this->local_params.emplace_back(std::make_shared<NullSE3Parameterization>());
                problem.AddParameterBlock(this->T_O_L.storage.data(), 12, this->local_params.back().get());
                param_ordering.AddElementToGroup(this->T_O_L.storage.data(), 1);
                this->T_O_L_added = true;
                problem.SetParameterBlockConstant(this->T_O_L.storage.data());
            }
            use_fixed = true;
        } else {
            if (feat_def.criteria.front().sel_pol == NEAR_ZERO) {
                this->local_params.emplace_back(std::make_shared<PlaneParameterization>());
            } else {
                this->local_params.emplace_back(std::make_shared<LineParameterization>());
            }
            problem.AddParameterBlock(track.geometry.data(), 6, this->local_params.back().get());
            if (opt_iter == 0) {
                problem.SetParameterBlockConstant(track.geometry.data());
            }
            param_ordering.AddElementToGroup(track.geometry.data(), 0);
        }
        this->buildResidualsFromMap(problem, track.static_mapping, track, f_idx, use_fixed);
        this->buildResidualsFromMap(problem, track.fluid_mapping, track, f_idx, use_fixed);
    }
}

void LaserOdom::buildResiduals(ceres::Problem &problem, ceres::ParameterBlockOrdering &param_ordering, int opt_iter) {
    size_t n_features = this->feature_extractor.param.feature_definitions.size();
    this->costs.clear();
    this->local_params.clear();
    this->loss_functions.clear();
    for (uint32_t state_id = 0; state_id < this->cur_trajectory.size(); ++state_id) {
        auto &state = this->cur_trajectory[state_id];
        if (false) {
            this->local_params.emplace_back(std::make_shared<NullSE3TranslationParameterization>());
            problem.AddParameterBlock(state.pose.storage.data(), 12, this->local_params.back().get());
            Vec<int> constant_indices{0, 1, 2};
            this->local_params.emplace_back(std::make_shared<ceres::SubsetParameterization>(6, constant_indices));
            problem.AddParameterBlock(state.vel.data(), 6, this->local_params.back().get());
        } else {
            this->local_params.emplace_back(std::make_shared<NullSE3Parameterization>());
            problem.AddParameterBlock(state.pose.storage.data(), 12, this->local_params.back().get());
            problem.AddParameterBlock(state.vel.data(), 6);
            state.vel.block<3,1>(0,0).setZero();
        }

        param_ordering.AddElementToGroup(state.pose.storage.data(), 1);
        param_ordering.AddElementToGroup(state.vel.data(), 1);

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
    for (uint32_t f_idx = 0; f_idx < n_features; ++f_idx) {
        this->trackResiduals(problem, param_ordering, f_idx, this->feature_tracks.at(f_idx), opt_iter);
//        this->rigidResiduals(problem, f_idx);
    }

    // add prior factor on starting velocity
//    if (this->prior_twist.sum() != 0.0) {
//        //        Mat6 sqrt_info = (this->twist_covar + this->prev_delta_t * this->param.Qc).inverse().sqrt();
//        Mat6 sqrt_info = (this->prev_delta_t * this->param.Qc).inverse().sqrt();
//        this->costs.emplace_back(new ceres::NormalPrior(sqrt_info, this->prior_twist));
//        problem.AddResidualBlock(this->costs.back().get(), nullptr, this->cur_trajectory.front().vel.data());
//    }

    // finally, just fix the first pose
//    if (opt_iter == 0) {
//        for (uint32_t i = 0; i < this->cur_trajectory.size(); ++i) {
//            if (i < this->cur_trajectory.size() - this->param.num_trajectory_states + 1) {
//                problem.SetParameterBlockConstant(this->cur_trajectory.at(i).pose.storage.data());
//            }
//            if (i < this->cur_trajectory.size() - 2 * (this->param.num_trajectory_states - 1)) {
//                problem.SetParameterBlockConstant(this->cur_trajectory.at(i).vel.data());
//            }
//        }
//    } else {
        problem.SetParameterBlockConstant(this->cur_trajectory.front().pose.storage.data());
//    }
}

void LaserOdom::resetTrajectory() {
    for (auto tra : this->cur_trajectory) {
        tra.pose.setIdentity();
        tra.vel.setZero();
    }
    this->prior_twist.setZero();
}

void LaserOdom::checkPosesNormalized() {
    for (auto& val : this->cur_trajectory) {
        const auto& R = val.pose.storage.block<3,3>(0,0);
        if (std::abs(R.determinant() - 1) > 1e-4 ||
            (R * R.transpose() - Mat3::Identity()).norm() > 1e-4) {
            LOG_ERROR("Pose not normalized, normalizing");
            val.pose.normalize();
        }
    }
}

void LaserOdom::checkTrackValidity() {
    for (uint32_t i = 0; i < this->feature_tracks.size(); ++i) {
        for (uint32_t j = 0; j < this->feature_tracks.at(i).size(); ++j) {
            const auto &track = this->feature_tracks.at(i).at(j);
            for (const auto &map : track.static_mapping) {
                if (this->feat_pts.at(map.scan_idx).at(i).dimension(1) < map.pt_idx) {
                    throw std::runtime_error("Invalid feature track");
                }
                if (map.state_id >= this->cur_trajectory.size()) {
                    throw std::runtime_error("Invalid state id in track");
                }
            }
            if (track.length > this->scan_stampsf.size()) {
                throw std::runtime_error("Track length too long");
            }
        }
    }
}


}  // namespace wave
