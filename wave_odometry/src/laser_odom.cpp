#include "wave/odometry/laser_odom.hpp"

namespace wave {

LaserOdom::LaserOdom(const LaserOdomParams params, const FeatureExtractorParams feat_params)
    : param(params), transformer(Transformer(TransformerParams())) {
    //    this->CSVFormat = new Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ");
    //
    //    auto n_ring = static_cast<size_t>(param.n_ring);
    //    this->feature_extractor.setParams(feat_params, n_ring);
    //    this->counters.resize(n_ring);
    //    std::fill(this->counters.begin(), this->counters.end(), 0);
    //
    //    this->cur_scan.resize(n_ring);
    //    this->signals.resize(n_ring);
    //
    //    for (uint32_t i = 0; i < n_ring; i++) {
    //        this->cur_scan.at(i) = Eigen::Tensor<float, 2>(4, this->MAX_POINTS);
    //        this->signals.at(i) = Eigen::Tensor<float, 2>(this->N_SIGNALS, this->MAX_POINTS);
    //    }
    //
    //    this->range_sensor = std::make_shared<RangeSensor>(param.sensor_params);
    //
    //    this->feat_pts.resize(this->N_FEATURES);
    //    this->prv_feature_points.resize(this->N_FEATURES);
    //    this->feature_corrs.resize(this->N_FEATURES);
    //    this->output_corrs.resize(this->N_FEATURES);
    //    this->feature_idx.resize(this->N_FEATURES);
    //    this->feature_association.resize(this->N_FEATURES);
    //    this->undis_features.resize(this->N_FEATURES);
    //    this->map_features.resize(this->N_FEATURES);
    //
    //    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
    //        this->feature_idx.at(i) = std::make_shared<kd_tree_t<double>>(
    //          3, this->prv_feature_points.at(i), nanoflann::KDTreeSingleIndexAdaptorParams(20));
    //        this->feature_corrs.at(i).resize(n_ring);
    //    }
    //
    //    if (this->param.num_trajectory_states < 2) {
    //        throw std::out_of_range("Number of parameter states must be at least 2");
    //    }
    //
    //    this->param_blocks.resize(this->param.num_trajectory_states);
    //    this->cur_difference.resize(this->param.num_trajectory_states - 1);
    //    for (auto &block : this->param_blocks) {
    //        block.setZero();
    //    }
    //
    //    // todo(ben) automatically get the scan
    //    perithis->trajectory_stamps.reserve(this->param.num_trajectory_states);
    //
    //    double step_size = 0.1 / (double) (this->param.num_trajectory_states - 1);
    //    for (uint32_t i = 0; i < this->param.num_trajectory_states; i++) {
    //        PoseVel unit;
    //        PoseVel unit2;
    //        unit2.pose.setIdentity();
    //        unit2.vel.setZero();
    //        unit.pose.setIdentity();
    //        unit.vel.setZero();
    //        this->current_twist.setZero();
    //        this->prior_twist.setZero();
    //        this->cur_trajectory.emplace_back(unit);
    //        this->prev_trajectory.emplace_back(unit2);
    //        this->trajectory_stamps.emplace_back(i * step_size);
    //        if (i > 0) {
    //            this->cv_vector.emplace_back(this->trajectory_stamps.at(i - 1),
    //                                         this->trajectory_stamps.at(i),
    //                                         nullptr,
    //                                         this->param.Qc,
    //                                         this->param.inv_Qc);
    //            this->cur_difference.at(i - 1).hat_multiplier.setZero();
    //            this->cur_difference.at(i - 1).candle_multiplier.setZero();
    //        }
    //    }
    //    this->sqrtinfo.setIdentity();
    //
    //
    //    if (params.output_trajectory) {
    //        long timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    //        this->file.open(std::to_string(timestamp) + "laser_odom_traj.txt");
    //    }
    //
    //    this->output_eigen.resize(6 * (1 + this->param.num_trajectory_states));
    //
    //    // Initial size of pre-allocated memory for jacobians
    //    this->PtLMem.resize(1000);
    //    this->PtPMem.resize(3000);
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

/**
 * Using projection matrix for solution remapping
 */
void LaserOdom::applyRemap() {
    VecX cur_diff;
    uint32_t offset = 0;

    cur_diff.resize((this->param.num_trajectory_states - 1) * 12, 1);
    offset = 1;

    for (uint32_t i = 0; i + offset < this->param.num_trajectory_states; i++) {
        cur_diff.block<6, 1>(12 * i, 0) =
          this->cur_trajectory.at(i + offset).pose.manifoldMinus(this->prev_trajectory.at(i + offset).pose);
        cur_diff.block<6, 1>(12 * i + 6, 0) =
          this->cur_trajectory.at(i + offset).vel - this->prev_trajectory.at(i + offset).vel;
    }

    if (this->param.plot_stuff) {
        plotMat(this->undistort_state.covar);
        MatX info = this->undistort_state.covar.inverse();
        plotMat(info);
        Eigen::SelfAdjointEigenSolver<MatX> eigs(info);
        plotVec(eigs.eigenvalues(), true);
        plotMat(eigs.eigenvectors());
    }

    MatX AtA = this->undistort_state.covar.inverse();

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigs(AtA);

    long cnt = 0;
    while (eigs.eigenvalues()(cnt) < this->param.min_eigen) {
        cnt++;
        if (cnt == eigs.eigenvectors().rows())
            break;
    }

    Eigen::MatrixXd Vu = eigs.eigenvectors().transpose();
    Vu.block(0, 0, cnt, Vu.cols()).setZero();
    MatX proj_mat = eigs.eigenvectors().transpose().inverse() * Vu;

    VecX mapped_diff = proj_mat * cur_diff;

    if (this->param.plot_stuff) {
        plotVec(cur_diff, true);
        plotVec(mapped_diff, true);
    }

    for (uint32_t i = 0; i + offset < this->param.num_trajectory_states; i++) {
        this->cur_trajectory.at(i + offset).pose = this->prev_trajectory.at(i + offset).pose;
        this->cur_trajectory.at(i + offset).pose.manifoldPlus(mapped_diff.block<6, 1>(12 * i, 0));

        this->cur_trajectory.at(i + offset).vel =
          this->cur_trajectory.at(i + offset).vel + mapped_diff.block<6, 1>(12 * i + 6, 0);
    }

    // set previous to current trajectory to update operating point
    this->copyTrajectory();
}

void LaserOdom::updateStoredFeatures() {
    // Perform a left rotation
    std::rotate(this->feat_pts.begin(), this->feat_pts.begin() + 1, this->feat_pts.end());
    std::rotate(this->feat_pts_T.begin(), this->feat_pts_T.begin() + 1, this->feat_pts_T.end());

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
        if (tick - this->prv_tick < -200) {  // tolerate minor nonlinearity error
            this->feature_extractor.getFeatures(this->cur_scan, this->signals, this->counters, this->indices);
            this->updateStoredFeatures();
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
            if (counters.at(pt.ring) >= this->MAX_POINTS) {
                throw std::out_of_range("Rebuild with higher max points");
            }
            this->cur_scan.at(pt.ring)(0, counters.at(pt.ring)) = pt.x;
            this->cur_scan.at(pt.ring)(1, counters.at(pt.ring)) = pt.y;
            this->cur_scan.at(pt.ring)(2, counters.at(pt.ring)) = pt.z;
            this->cur_scan.at(pt.ring)(3, counters.at(pt.ring)) =
              std::chrono::duration_cast<std::chrono::seconds>(stamp - this->scan_stamps_chrono.front()).count();

            this->signals.at(pt.ring)(0, counters.at(pt.ring)) = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            this->signals.at(pt.ring)(1, counters.at(pt.ring)) = pt.intensity;

            this->counters.at(pt.ring)++;
        }

        this->prv_tick = tick;
}

void LaserOdom::rollover(TimeType stamp) {
    //    for (uint32_t i = 0; i + 1 < this->param.n_window; i++) {
    //        std::swap(this->feat_pts.at(i), this->feat_pts.at(i + 1));
    //        std::swap(this->scan_stamps.at(i), this->scan_stamps.at(i + 1));
    //    }
    //    this->scan_stamps.back() = stamp;
    //
    //    this->buildTrees();
    //    for (unlong i = 0; i < this->param.n_ring; i++) {
    //        this->counters.at(i) = 0;
    //    }
    //    if (!this->initialized) {
    //        // This check is to avoid initializing against a partial scan.
    //        if (!this->full_revolution) {
    //            this->full_revolution = true;
    //            return;
    //        }
    //        size_t feature_count = 0;
    //        for (uint32_t i = 0; i < this->N_FEATURES; i++) {
    //            feature_count += this->prv_feature_points.at(i).points.size();
    //        }
    //        if (feature_count >= (size_t)(this->param.min_features)) {
    //            this->initialized = true;
    //        }
    //    }
    //    this->prior_twist = this->cur_trajectory.back().vel;
    //    this->cur_trajectory.back().pose.transformInverse(this->inv_prior_pose);
    //
    //    this->cur_trajectory.front().pose = this->cur_trajectory.back().pose;
    //
    //    for (uint32_t i = 1; i < this->param.num_trajectory_states; i++) {
    //        this->cur_trajectory.at(i).pose = this->cur_trajectory.at(i - 1).pose;
    //        this->cur_trajectory.at(i).pose.manifoldPlus(
    //          (this->param.scan_period / (this->param.num_trajectory_states - 1)) * this->cur_trajectory.back().vel);
    //    }
    //    // Now previous trajectory will hold the "motion generated" trajectory
    //    this->copyTrajectory();
    //    this->updateDifferences();
}

bool LaserOdom::outOfBounds(const Vec3 &query, const uint32_t &f_idx, const std::vector<size_t> &index) {
    //    Eigen::Map<const Vec3> pA(this->prv_feature_points.at(f_idx).points.at(index.at(0)).data());
    //    Eigen::Map<const Vec3> pB(this->prv_feature_points.at(f_idx).points.at(index.at(1)).data());
    //    if (this->feature_residuals.at(f_idx) == PointToPlane) {
    //        return false;
    //    } else {
    //        Vec3 AB = pB - pA;
    //        Vec3 Aq = query - pA;
    //
    //        double eta = Aq.dot(AB) / AB.dot(AB);
    //
    //        if (eta < -this->param.max_extrapolation || eta > (1.0 + this->param.max_extrapolation)) {
    //            return true;
    //        }
    //    }
    //    return false;
}

bool LaserOdom::runOptimization(ceres::Problem &problem) {
    // Add motion residuals
    // The prior for the pose at the end of the previous trajectory, the prior for twist is the twist at the end of the
    // previous
    // trajectory
    //    Transformation<> identity;
    //    if (this->param.motion_prior) {
    //        this->cv_vector.at(0).calculateLinInvCovariance();
    //
    //        Vec12 op_error;
    //
    //        T_TYPE diff;
    //        this->cur_trajectory.begin()->pose.compose(this->inv_prior_pose, diff);
    //        op_error.block<6, 1>(0, 0) = diff.logMap();
    //        op_error.block<6, 1>(6, 0) = this->cur_trajectory.begin()->vel - this->prior_twist;
    //
    //        // expect the correction to be the negative of the error
    //        ceres::CostFunction *prior_cost = new ceres::NormalPrior(this->cv_vector.at(0).inv_covar.sqrt(),
    //        -op_error);
    //
    //        problem.AddResidualBlock(prior_cost, nullptr, this->param_blocks.at(0).data());
    //    }
    //
    //    for (uint32_t i = 0; i < this->param.num_trajectory_states; i++) {
    //        if (i + 1 < this->param.num_trajectory_states) {
    //            this->cv_vector.at(i).calculateLinInvCovariance();
    //
    //            Vec12 op_error;
    //
    //            op_error.block<6, 1>(0, 0) =
    //              this->cur_trajectory.at(i + 1).pose.manifoldMinus(this->cur_trajectory.at(i).pose) -
    //              (this->cv_vector.at(i).tkp1 - this->cv_vector.at(i).tk) * this->cur_trajectory.at(i).vel;
    //            op_error.block<6, 1>(6, 0) = this->cur_trajectory.at(i + 1).vel - this->cur_trajectory.at(i).vel;
    //
    //            ceres::CostFunction *motion_cost = new wave_optimization::ConstantVelocityPrior(
    //              this->cv_vector.at(i).inv_covar.sqrt(), op_error, this->cv_vector.at(i).tkp1 -
    //              this->cv_vector.at(i).tk);
    //
    //            problem.AddResidualBlock(
    //              motion_cost, nullptr, this->param_blocks.at(i).data(), this->param_blocks.at(i + 1).data());
    //        }
    //    }
    //
    //    // now loop over each type of feature, and generate residuals for each
    //    Eigen::Matrix<double, 12, 12> candle, hat;
    //
    //    uint32_t k, kp1;
    //    double tau;
    //
    //    std::vector<size_t> ret_indices;
    //    std::vector<double> out_dist_sqr;
    //
    //    uint64_t cur_PtL_idx = 0, cur_PtP_idx = 0;

    //                    ceres::LossFunction *p_LossFunction = new BisquareLoss(this->param.robust_param);
    //                    std::vector<uint64_t> corr_list;
    //                    corr_list.emplace_back(pt_cntr);
    //                    for (auto index : ret_indices) {
    //                        corr_list.emplace_back(index);
    //                        this->feature_association.at(i).at(index).second = AssociationStatus::CORRESPONDED;
    //                    }
    //                    this->feature_corrs.at(i).at(j).emplace_back(corr_list);
    //
    //                    problem.AddResidualBlock(cost_function,
    //                                             p_LossFunction,
    //                                             this->param_blocks.at(k).data(),
    //                                             this->param_blocks.at(kp1).data());
    //
    //                    if (pTl_cost_function) {
    //                        cur_PtL_idx++;
    //                    } else if (pTPl_cost_function) {
    //                        cur_PtP_idx++;
    //                    }
    //                }
    //            }
    //        }
    //    }
    //
    //    if (this->param.lock_first) {
    //        this->covar.resize(12 * (this->param.num_trajectory_states - 1), 12 * (this->param.num_trajectory_states -
    //        1));
    //    } else {
    //        this->covar.resize(12 * this->param.num_trajectory_states, 12 * this->param.num_trajectory_states);
    //    }
    //
    //    for (auto &block : this->param_blocks) {
    //        problem.AddParameterBlock(block.data(), 12, nullptr);
    //    }
    //
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
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
        if (this->param.solution_remapping) {
            this->applyRemap();
        }
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
    long start_offset = this->feat_pts.back().at(feat_id).dimension(1);
    long new_feat_cnt = 0;
    for (uint32_t j = 0; j < idx.cols(); ++j) {
        std::vector<uint32_t> matches;
        double min_elev = 0.0;
        double max_elev = 0.0;
        bool wide_spread = false;
        /// Each row is a nearest neighbour
        for(uint32_t i = 0; i < idx.rows(); ++i) {
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
                    if(new_elev > max_elev)
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
        /// add to residual if error against current landmark is low
        if (matches.size() == knn) {
            for (const auto &elem : matches) {
                const auto &geometry = this->feature_tracks.at(feat_id).at(j).geometry;
                const auto &pt = this->cur_feat_map.at(feat_id).block<3, 1>(0, elem);
                Vec3 diff = (pt.cast<double>() - geometry.block<3, 1>(3,0));
                Vec1 error;
                if (this->feature_residuals.at(feat_id) == PointToLine) {
                    Vec3 err =  (diff - geometry.block<3, 1>(0,0) * (diff.transpose() * geometry.block<3, 1>(0,0)));
                    error = err.transpose() * err;
                } else {
                    error = (diff.transpose() * geometry.block<3, 1>(0,0));
                    error = error.transpose() * error;
                }
                // copy candidate point into the feature point set (both original and transformed) if it is not already
                // used in another residual
                if (error(0) < sqrt(this->param.max_residual_val) &&
                    this->cur_feat_idx.at(feat_id).at(elem) == -1) {

                    this->cur_feat_idx.at(feat_id).at(elem) = j;

                    Eigen::array<int, 2> offsets_new = {0, static_cast<int>(new_feat_cnt)};
                    Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(elem)};
                    Eigen::array<int, 2> extents = {4, 1};

                    new_feat_points.slice(offsets_new, extents) = this->cur_feature_candidates.at(feat_id).slice(offsets_candidate, extents);

                    this->feature_tracks.at(feat_id).at(j).mapping.emplace_back(new_feat_cnt, this->param.n_window - 1);
                    std::vector<uint32_t> states(4);
                    auto bnd = std::upper_bound(this->trajectory_stamps.begin(), this->trajectory_stamps.end(), new_feat_points(3, new_feat_cnt));
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
}

void LaserOdom::createNewFeatureTracks(const Eigen::MatrixXi &indices, const Eigen::MatrixXf &dist, uint32_t feat_id) {

}

bool LaserOdom::match() {
    T_TYPE last_transform;
    auto &ref = this->cur_trajectory.back().pose;

    ceres::Problem problem;

    for (int op = 0; op < this->param.opt_iters; op++) {
        if (op > 0) {
            last_transform = ref;
        }
        for (uint32_t i = 0; i < this->scan_stamps_chrono.size(); i++) {
            this->scan_stampsf.at(i) = std::chrono::duration_cast<std::chrono::seconds>(this->scan_stamps_chrono.at(i) - this->scan_stamps_chrono.front()).count();
        }
        this->transformer.update(this->cur_trajectory, this->scan_stampsf);

        for (uint32_t j = 0; j < this->N_FEATURES; j++) {
            /// 1. Transform all features to the start of the window
            for (uint32_t i = 0; i < this->param.n_window; i++) {
                auto &feat = this->feat_pts.at(i).at(j);
                auto &featT = this->feat_pts_T.at(i).at(j);
                this->transformer.transformToStart(feat, featT);
            }
            /// 1.5 Transform all candidate points to the start of the window
            this->transformer.transformToStart(this->cur_feature_candidates.at(j), this->cur_feature_candidatesT.at(j));
            this->transformer.transformToStart(this->prev_feature_candidates.at(j), this->prev_feature_candidatesT.at(j));
            auto &cfeat = this->cur_feature_candidatesT.at(j);
            this->cur_feat_map.at(j) = Eigen::Map<Eigen::MatrixXf>(cfeat.data(), cfeat.dimension(0), cfeat.dimension(1));

            auto &pfeat = this->prev_feature_candidatesT.at(j);
            this->prev_feat_map.at(j) = Eigen::Map<Eigen::MatrixXf>(pfeat.data(), pfeat.dimension(0), pfeat.dimension(1));

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
            this->curm1_kd_idx.at(j)->knn(this->cur_feat_map.at(j), nn_idx, nn_dist, 5, 0.1, Nabo::NNSearchF::SORT_RESULTS);
            this->createNewFeatureTracks(nn_idx, nn_dist, j);

            /// 6. Merge feature tracks (optional)

            /// 7. Build Feature Residuals


            if (!this->runOptimization(problem))
                return false;

        }

        //        for (uint32_t m_idx = 0; m_idx < this->param.n_window; m_idx++) {
        //            Eigen::Matrix3f covZ;
        //            if (this->findCorrespondingPoints(query, i, &ret_indices)) {
        //                // check if there is enough memory for next cost function
        //                if (cur_PtL_idx == this->PtLMem.size()) {
        //                    LOG_ERROR("Pre allocated point to line memory block is too small, reseting");
        //                    return false;
        //                }
        //                if (cur_PtP_idx == this->PtPMem.size()) {
        //                    LOG_ERROR("Pre allocated point to plane memory block is too small, reseting");
        //                    return false;
        //                }
        //                if (this->param.no_extrapolation && this->outOfBounds(query, i, ret_indices)) {
        //                    break;
        //                }
        //                this->cv_vector.at(k).tau = &tau;
        //                this->cv_vector.at(k).calculateStuff(hat, candle);
        //                ceres::CostFunction *cost_function;
        //                wave_optimization::SE3PointToLineGP *pTl_cost_function = nullptr;
        //                wave_optimization::SE3PointToPlaneGP *pTPl_cost_function = nullptr;
        //                double rescale;
        //                switch (this->feature_definitions.at(i).residual) {
        //                    case PointToLine:
        //                        if (this->param.treat_lines_as_planes) {
        //                            this->PtPMem.at(cur_PtP_idx).hat = hat.block<6, 12>(0, 0);
        //                            this->PtPMem.at(cur_PtP_idx).candle = candle.block<6, 12>(0, 0);
        //                            this->PtPMem.at(cur_PtP_idx).T0_pt = query;
        //                            pTPl_cost_function = new wave_optimization::SE3PointToPlaneGP(
        //                              this->prv_feature_points.at(i).points.at(ret_indices.at(0)).data(),
        //                              this->prv_feature_points.at(i).points.at(ret_indices.at(1)).data(),
        //                              zero_pt,
        //                              this->PtPMem.at(cur_PtP_idx),
        //                              covZ.cast<double>(),
        //                              this->param.use_weighting);
        //                            residuals.resize(1);
        //                            rescale = pTPl_cost_function->weight;
        //                            cost_function = pTPl_cost_function;
        //                        } else {
        //                            this->PtLMem.at(cur_PtL_idx).hat = hat.block<6, 12>(0, 0);
        //                            this->PtLMem.at(cur_PtL_idx).candle = candle.block<6, 12>(0, 0);
        //                            this->PtLMem.at(cur_PtP_idx).T0_pt = query;
        //                            pTl_cost_function = new wave_optimization::SE3PointToLineGP(
        //                              this->prv_feature_points.at(i).points.at(ret_indices.at(0)).data(),
        //                              this->prv_feature_points.at(i).points.at(ret_indices.at(1)).data(),
        //                              this->PtLMem.at(cur_PtL_idx),
        //                              covZ.cast<double>(),
        //                              this->param.use_weighting);
        //                            residuals.resize(2);
        //                            rescale = pTl_cost_function->weight_matrix.trace();
        //                            cost_function = pTl_cost_function;
        //                        }
        //                        break;
        //                    case PointToPlane:
        //                        this->PtPMem.at(cur_PtP_idx).hat = hat.block<6, 12>(0, 0);
        //                        this->PtPMem.at(cur_PtP_idx).candle = candle.block<6, 12>(0, 0);
        //                        this->PtPMem.at(cur_PtP_idx).T0_pt = query;
        //                        pTPl_cost_function = new wave_optimization::SE3PointToPlaneGP(
        //                          this->prv_feature_points.at(i).points.at(ret_indices.at(0)).data(),
        //                          this->prv_feature_points.at(i).points.at(ret_indices.at(1)).data(),
        //                          this->prv_feature_points.at(i).points.at(ret_indices.at(2)).data(),
        //                          this->PtPMem.at(cur_PtP_idx),
        //                          covZ.cast<double>(),
        //                          this->param.use_weighting);
        //                        residuals.resize(1);
        //                        rescale = pTPl_cost_function->weight;
        //                        cost_function = pTPl_cost_function;
        //                        break;
        //                    default: continue;
        //                }
        //
        //                parameters[0] = this->param_blocks.at(k).data();
        //                parameters[1] = this->param_blocks.at(kp1).data();
        //
        //                if (!cost_function->Evaluate(parameters, residuals.data(), nullptr)) {
        //                    LOG_ERROR("Cost function did not evaluate");
        //                    delete cost_function;
        //                    continue;
        //                }
        //                rescale *= rescale;
        //
        //                if (norm(residuals) > rescale * this->param.max_residual_val) {
        //                    delete cost_function;
        //                    continue;
        //                }
        //
        //                ceres::LossFunction *p_LossFunction = new BisquareLoss(this->param.robust_param);
        //                std::vector<uint64_t> corr_list;
        //                corr_list.emplace_back(pt_cntr);
        //                for (auto index : ret_indices) {
        //                    corr_list.emplace_back(index);
        //                    this->feature_association.at(i).at(index).second = AssociationStatus::CORRESPONDED;
        //                }
        //                this->feature_corrs.at(i).at(j).emplace_back(corr_list);
        //
        //                problem.AddResidualBlock(
        //                  cost_function, p_LossFunction, this->param_blocks.at(k).data(),
        //                  this->param_blocks.at(kp1).data());
        //
        //                if (pTl_cost_function) {
        //                    cur_PtL_idx++;
        //                } else if (pTPl_cost_function) {
        //                    cur_PtP_idx++;
        //                }
        //                /// 5. Solve
        //                if (!this->runOptimization(problem)) {
        //                    return false;
        //                }
        //                if (op > 0 && ref.isNear(last_transform, this->param.diff_tol)) {
        //                    return true;
        //                }
        //            }
        //            LOG_INFO("Problem did not converge within allowed iterations");
        //            return true;
        //        }
    }
    return true;
}

void LaserOdom::resetTrajectory() {
    for (auto tra : this->cur_trajectory) {
        tra.pose.setIdentity();
        tra.vel.setZero();
    }
    this->prior_twist.setZero();
}

}  // namespace wave
