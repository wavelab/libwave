#include <ceres/gradient_checker.h>
#include "wave/odometry/LaserOdom.hpp"
#include "wave/odometry/weighting_functions.hpp"

namespace wave {

double l2length(const double *const vec, uint16_t length) {
    double retval = 0;
    for (uint16_t i = 0; i < length; i++) {
        retval += vec[i] * vec[i];
    }
    return retval;
}

double norm(const std::vector<double> &vec) {
    double retval = 0;
    for (size_t i = 0; i < vec.size(); i++) {
        retval += vec[i] * vec[i];
    }
    return std::sqrt(retval);
}

LaserOdom::LaserOdom(const LaserOdomParams params) : param(params) {
    this->CSVFormat = new Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ");

    auto n_ring = static_cast<size_t>(param.n_ring);
    this->cur_scan.resize(n_ring);
    this->kernels.resize(this->N_SCORES);
    this->kernels.at(0) = std::make_shared<Eigen::Tensor<double, 1>>(11);
    memcpy(this->kernels.at(0)->data(), loam_kernel, 88);
    this->kernels.at(1) = std::make_shared<Eigen::Tensor<double, 1>>(11);
    memcpy(this->kernels.at(1)->data(), LoG_kernel, 88);
    this->kernels.at(2) = std::make_shared<Eigen::Tensor<double, 1>>(9);
    memcpy(this->kernels.at(2)->data(), FoG_kernel, 72);

    this->range_sensor = std::make_shared<RangeSensor>(param.sensor_params);

    // Define features
    std::vector<Criteria> edge_high, edge_low, flat, edge_int_high, edge_int_low;
    edge_high.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::HIGH_POS, &(param.edge_tol)});
    edge_low.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::HIGH_NEG, &(param.edge_tol)});
    flat.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.flat_tol)});
    edge_int_high.emplace_back(Criteria{Kernel::FOG, SelectionPolicy::HIGH_POS, &(param.int_edge_tol)});
    edge_int_high.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_low.emplace_back(Criteria{Kernel::FOG, SelectionPolicy::HIGH_POS, &(param.int_edge_tol)});
    edge_int_low.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});

    this->feature_definitions.emplace_back(FeatureDefinition{edge_high, ResidualType::PointToLine, &(param.n_edge)});
    this->feature_definitions.emplace_back(FeatureDefinition{edge_low, ResidualType::PointToLine, &(param.n_edge)});
    this->feature_definitions.emplace_back(FeatureDefinition{flat, ResidualType::PointToPlane, &(param.n_flat)});
    this->feature_definitions.emplace_back(
      FeatureDefinition{edge_int_high, ResidualType::PointToLine, &(param.n_int_edge)});
    this->feature_definitions.emplace_back(
      FeatureDefinition{edge_int_low, ResidualType::PointToLine, &(param.n_int_edge)});

    this->signals.resize(this->N_SIGNALS);
    this->scores.resize(this->N_SCORES);

    this->valid_pts.resize(this->N_FEATURES);
    this->filtered_scores.resize(this->N_FEATURES);
    this->feature_points.resize(this->N_FEATURES);
    this->prv_feature_points.resize(this->N_FEATURES);
    this->feature_corrs.resize(this->N_FEATURES);
    this->output_corrs.resize(this->N_FEATURES);
    this->feature_idx.resize(this->N_FEATURES);
    this->feature_association.resize(this->N_FEATURES);
    this->undis_features.resize(this->N_FEATURES);
    this->map_features.resize(this->N_FEATURES);

    for (uint32_t i = 0; i < this->N_SIGNALS; i++) {
        this->signals.at(i).resize(n_ring);
    }

    for (uint32_t i = 0; i < this->N_SCORES; i++) {
        this->scores.at(i).resize(n_ring);
    }

    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
        this->valid_pts.at(i).resize(n_ring);
        this->feature_points.at(i).resize(n_ring);
        this->feature_idx.at(i) =
          std::make_shared<kd_tree_t>(3, this->prv_feature_points.at(i), nanoflann::KDTreeSingleIndexAdaptorParams(20));
        this->filtered_scores.at(i).resize(n_ring);
        this->feature_corrs.at(i).resize(n_ring);
    }

    if (this->param.num_trajectory_states < 2) {
        throw std::out_of_range("Number of parameter states must be at least 2");
    }

    // todo(ben) automatically get the scan period
    this->trajectory_stamps.reserve(this->param.num_trajectory_states);

    double step_size = 0.1 / (double) (this->param.num_trajectory_states - 1);
    for (uint32_t i = 0; i < this->param.num_trajectory_states; i++) {
        Trajectory unit;
        unit.pose.setIdentity();
        unit.twist.setZero();
        this->cur_trajectory.emplace_back(unit);
        this->trajectory_stamps.emplace_back((double) (i) *step_size);
        if (i > 0) {
            this->cv_vector.emplace_back(
              this->trajectory_stamps.at(i - 1), this->trajectory_stamps.at(i), nullptr, this->param.Qc, this->param.inv_Qc);
        }
    }

    this->sqrtinfo.setIdentity();

    if (params.visualize) {
        this->display = new PointCloudDisplay("laser odom");
        this->display->startSpin();
        // Allocate space for visualization clouds
        this->prev_viz = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>);
        this->cur_viz = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>);
    }

    if (params.output_trajectory) {
        long timestamp = std::chrono::system_clock::now().time_since_epoch().count();
        this->file.open(std::to_string(timestamp) + "laser_odom_traj.txt");
    }

    this->covariance_blocks.push_back(
      std::make_pair(this->cur_trajectory.back().pose.getInternalMatrix().derived().data(),
                     this->cur_trajectory.back().pose.getInternalMatrix().derived().data()));
    this->covariance_blocks.push_back(std::make_pair(
      this->cur_trajectory.back().pose.getInternalMatrix().derived().data(), this->cur_trajectory.back().twist.data()));
    this->covariance_blocks.push_back(
      std::make_pair(this->cur_trajectory.back().twist.data(), this->cur_trajectory.back().twist.data()));
}

/// tau is the time of the point
void LaserOdom::getTransformIndices(const uint32_t &tick, uint32_t &start, uint32_t &end, double &tau) {
    // This will round down to provide first transform index
    start = ((tick * (param.num_trajectory_states - 1)) / param.max_ticks);
    end = start + 1;
    tau = (tick * this->param.scan_period) / (this->param.max_ticks);
}

void LaserOdom::flagNearbyPoints(const unlong f_idx, const unlong ring, const unlong p_idx) {
    for (unlong j = 0; j < this->param.key_radius; j++) {
        if (p_idx + j + 1 >= this->valid_pts.at(f_idx).at(ring).size()) {
            break;
        }
        this->valid_pts.at(f_idx).at(ring).at(p_idx + j + 1) = false;
    }
    for (unlong j = 0; j < this->param.key_radius; j++) {
        if (p_idx < j + 1) {
            break;
        }
        this->valid_pts.at(f_idx).at(ring).at(p_idx - j - 1) = false;
    }
}

void LaserOdom::transformToStart(
  const double *const pt, const uint16_t tick, double *output, uint32_t &k, uint32_t &kp1, double &tau) {
    this->getTransformIndices(tick, k, kp1, tau);

    T_Type transform;

    Eigen::Matrix<double, 12, 12> hat, candle;
    this->cv_vector.at(k).tau = &tau;
    this->cv_vector.at(k).calculateStuff(hat, candle);

    transform = transform.interpolate(this->cur_trajectory.at(k).pose,
                                              this->cur_trajectory.at(kp1).pose,
                                              this->cur_trajectory.at(k).twist,
                                              this->cur_trajectory.at(kp1).twist,
                                              hat.block<6, 12>(0, 0),
                                              candle.block<6, 12>(0, 0));

    Eigen::Map<const Vec3> Vecpt(pt, 3, 1);
    Vec3 transformed = transform.transform(Vecpt);
    Eigen::Map<Vec3>(output, 3, 1) = transformed;
}

void LaserOdom::transformToStart(const double *const pt, const uint16_t tick, double *output) {
    uint32_t k, kp1;
    double tau;
    this->getTransformIndices(tick, k, kp1, tau);

    Eigen::Matrix<double, 12, 12> hat, candle;
    this->cv_vector.at(k).tau = &tau;
    this->cv_vector.at(k).calculateStuff(hat, candle);

    T_Type transform;
    transform = transform.interpolate(this->cur_trajectory.at(k).pose,
                                                               this->cur_trajectory.at(kp1).pose,
                                                               this->cur_trajectory.at(k).twist,
                                                               this->cur_trajectory.at(kp1).twist,
                                                               hat.block<6, 12>(0, 0),
                                                               candle.block<6, 12>(0, 0));

    Eigen::Map<const Vec3> Vecpt(pt, 3, 1);
    Vec3 transformed = transform.transform(Vecpt);
    Eigen::Map<Vec3>(output, 3, 1) = transformed;
}

void LaserOdom::transformToEnd(const double *const pt, const uint16_t tick, double *output) {
    uint32_t k, kp1;
    double tau;
    this->getTransformIndices(tick, k, kp1, tau);

    Eigen::Matrix<double, 12, 12> hat, candle;
    this->cv_vector.at(k).tau = &tau;
    this->cv_vector.at(k).calculateStuff(hat, candle);

    T_Type transform;
    transform = transform.interpolate(this->cur_trajectory.at(k).pose,
                                                               this->cur_trajectory.at(kp1).pose,
                                                               this->cur_trajectory.at(k).twist,
                                                               this->cur_trajectory.at(kp1).twist,
                                                               hat.block<6, 12>(0, 0),
                                                               candle.block<6, 12>(0, 0));

    Eigen::Map<const Vec3> Vecpt(pt, 3, 1);
    Vec3 transformed = transform.inverseTransform(Vecpt);
    Eigen::Map<Vec3>(output, 3, 1) = transformed;
}

float LaserOdom::l2sqrd(const PCLPointXYZIT &p1, const PCLPointXYZIT &p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}

float LaserOdom::l2sqrd(const PCLPointXYZIT &pt) {
    return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
}

PCLPointXYZIT LaserOdom::scale(const PCLPointXYZIT &pt, const float scale) {
    PCLPointXYZIT p;
    p.x = pt.x * scale;
    p.y = pt.y * scale;
    p.z = pt.z * scale;
    p.intensity = pt.intensity;
    p.tick = pt.tick;
    return p;
}

void LaserOdom::updateParams(const LaserOdomParams new_params) {
    this->param = new_params;
}

LaserOdomParams LaserOdom::getParams() {
    return this->param;
}

LaserOdom::~LaserOdom() {
    if (this->param.visualize) {
        this->display->stopSpin();
        delete this->display;
    }
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
    this->f_output = output_function;
    this->output_thread = std::unique_ptr<std::thread>(new std::thread(&LaserOdom::spinOutput, this));
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

void LaserOdom::undistort() {
    this->undistorted_cld.clear();
    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
        this->undis_features.at(i).clear();
        this->map_features.at(i).resize(this->prv_feature_points.at(i).points.size());
        this->output_corrs.at(i).clear();
    }

    for (uint16_t r_idx = 0; r_idx < this->param.n_ring; r_idx++) {
        for (PCLPointXYZIT pt : this->cur_scan.at(r_idx)) {
            double point[3] = {pt.x, pt.y, pt.z};
            double u_pt[3];
            this->transformToStart(point, pt.tick, u_pt);
            pcl::PointXYZI op_pt;
            op_pt.x = (float) u_pt[0];
            op_pt.y = (float) u_pt[1];
            op_pt.z = (float) u_pt[2];
            op_pt.intensity = pt.intensity;
            this->undistorted_cld.push_back(op_pt);
        }
        for (uint32_t j = 0; j < this->N_FEATURES; j++) {
            for (uint32_t i = 0; i < this->feature_points.at(j).at(r_idx).size(); i++) {
                double point[3] = {this->feature_points.at(j).at(r_idx).at(i).pt[0],
                                   feature_points.at(j).at(r_idx).at(i).pt[1],
                                   feature_points.at(j).at(r_idx).at(i).pt[2]};
                double u_pt[3];
                this->transformToStart(point, feature_points.at(j).at(r_idx).at(i).tick, u_pt);
                pcl::PointXYZ op_pt;
                op_pt.x = (float) u_pt[0];
                op_pt.y = (float) u_pt[1];
                op_pt.z = (float) u_pt[2];
                this->undis_features.at(j).push_back(op_pt);
            }

            for (uint32_t c_idx = 0; c_idx < this->feature_corrs.at(j).at(r_idx).size(); c_idx++) {
                auto &corr_list = this->feature_corrs.at(j).at(r_idx).at(c_idx);
                std::vector<double> undis(3 * (corr_list.size() + 1));
                this->transformToStart(&(this->feature_points.at(j).at(r_idx).at(corr_list.at(0)).pt[0]),
                                       this->feature_points.at(j).at(r_idx).at(corr_list.at(0)).tick,
                                       &(undis[undis.size() - 3]));
                memcpy(undis.data(), this->feature_points.at(j).at(r_idx).at(corr_list.at(0)).pt, 24);
                for (uint32_t k = 1; k < corr_list.size(); k++) {
                    memcpy(undis.data() + 3 * k, this->prv_feature_points.at(j).points.at(corr_list.at(k)).data(), 24);
                }

                this->output_corrs.at(j).emplace_back(std::vector<double>(undis.begin(), undis.end()));
            }
        }
    }

    if (this->param.output_correspondences) {
        long timestamp = std::chrono::system_clock::now().time_since_epoch().count();
        for (uint32_t i = 0; i < this->N_FEATURES; i++) {
            ofstream cur_file;
            cur_file.open(std::to_string(timestamp) + "feature_" + std::to_string(i) + "_cor.txt");
            for (auto vec : this->output_corrs.at(i)) {
                for (auto val : vec) {
                    cur_file << std::to_string(val) << " ";
                }
                cur_file << std::endl;
            }
            cur_file.close();
        }
    }

    for (uint32_t j = 0; j < this->N_FEATURES; j++) {
        for (size_t i = 0; i < this->prv_feature_points.at(j).points.size(); i++) {
            // stupid friggin pcl and its floats
            this->map_features.at(j).points.at(i).x = (float) this->prv_feature_points.at(j).points.at(i).at(0);
            this->map_features.at(j).points.at(i).y = (float) this->prv_feature_points.at(j).points.at(i).at(1);
            this->map_features.at(j).points.at(i).z = (float) this->prv_feature_points.at(j).points.at(i).at(2);
        }
    }
}

void LaserOdom::addPoints(const std::vector<PointXYZIR> &pts, const int tick, TimeType stamp) {
    if ((tick - this->prv_tick) < -200) {  // tolerate minor nonlinearity error
        this->generateFeatures();          // generate features on the current scan
        if (this->initialized) {           // there is a set of previous features from
                                           // last scan
            T_Type last_transform;
            auto &ref = this->cur_trajectory.back().pose;

            for (int i = 0; i < this->param.opt_iters; i++) {
                if (i > 0) {
                    memcpy(last_transform.getInternalMatrix().derived().data(),
                           ref.getInternalMatrix().derived().data(),
                           96);
                }
                if (!this->match()) {
                    return;
                }
                if (i > 0) {
                    if (ref.isNear(last_transform, this->param.diff_tol)) {
                        break;
                    }
                }
            }

            for (uint32_t j = 0; j < this->param.num_trajectory_states; j++) {
                std::cout << this->cur_trajectory.at(j).pose.getInternalMatrix().format(*(this->CSVFormat)) << std::endl
                          << std::endl;
                std::cout << this->cur_trajectory.at(j).twist.format(*(this->CSVFormat)) << std::endl << std::endl;
            }

            if (this->param.output_trajectory) {
                this->file << this->cur_trajectory.back().pose.getInternalMatrix().format(*(this->CSVFormat))
                           << std::endl;
            }
            if (this->param.visualize) {
                this->updateViz();
            }
            if (this->output_thread) {
                {
                    std::unique_lock<std::mutex> lk(this->output_mutex);
                    if (this->fresh_output) {
                        // data from last time hasn't been consumed yet
                        LOG_ERROR("Overwriting previous output");
                    }
                    this->undistorted_stamp = this->prv_time;
                    memcpy(this->undistort_transform.getInternalMatrix().derived().data(),
                           this->cur_trajectory.back().pose.getInternalMatrix().derived().data(),
                           96);

                    this->undistort();
                    this->fresh_output = true;
                }
                this->output_condition.notify_one();
            }
        }
        this->rollover(stamp);
    }

    for (PointXYZIR pt : pts) {
        PCLPointXYZIT p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        p.tick = (uint16_t) tick;
        this->cur_scan.at(pt.ring).push_back(this->applyIMU(p));
        this->signals[0].at(pt.ring).push_back(std::sqrt(l2sqrd(p)));
        this->signals[1].at(pt.ring).push_back((double) p.intensity);
    }

    this->prv_tick = tick;
}

void LaserOdom::updateViz() {
    // populate prev with points stored in kd tree
    // These are already transformed to the start of the current scan
    this->display->removeAll();
    this->prev_viz->clear();
    this->cur_viz->clear();

    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
        for (auto iter = this->prv_feature_points.at(i).points.begin();
             iter != this->prv_feature_points.at(i).points.end();
             iter++) {
            pcl::PointXYZI pt;
            pt.x = (float) (*iter).at(0);
            pt.y = (float) (*iter).at(1);
            pt.z = (float) (*iter).at(2);
            pt.intensity = 1 + i;
            this->prev_viz->push_back(pt);
        }

        for (uint32_t j = 0; j < this->param.n_ring; j++) {
            for (auto pt : this->feature_points.at(i).at(j)) {
                double T_pt[3];
                this->transformToStart(pt.pt, pt.tick, T_pt);
                pcl::PointXYZI point;
                point.x = T_pt[0];
                point.y = T_pt[1];
                point.z = T_pt[2];
                point.intensity = 10 + i;
                this->prev_viz->push_back(point);
            }
        }
    }

    this->display->addPointcloud(this->prev_viz, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds((int) (10 * this->param.scan_period)));
}

PCLPointXYZIT LaserOdom::applyIMU(const PCLPointXYZIT &p) {
    // for now don't transform
    PCLPointXYZIT pt;
    pt = p;
    return pt;
}

void LaserOdom::rollover(TimeType stamp) {
    this->prv_time = this->cur_time;
    this->cur_time = stamp;
    // this->resetIMU(stamp);
    this->buildTrees();
    for (unlong i = 0; i < this->param.n_ring; i++) {
        for (uint32_t j = 0; j < this->N_SIGNALS; j++) {
            this->signals.at(j).at(i).clear();
        }
        this->cur_scan.at(i).clear();
    }
    if (!this->initialized) {
        size_t feature_count = 0;
        for (uint32_t i = 0; i < this->N_FEATURES; i++) {
            feature_count += this->prv_feature_points.at(i).points.size();
        }
        if (feature_count >= (size_t)(this->param.n_edge + this->param.n_flat)) {
            this->initialized = true;
        }
    }
    this->previous_twist = this->cur_trajectory.back().twist;
    this->cur_trajectory.front().pose.setIdentity();
    for (uint32_t i = 1; i < this->param.num_trajectory_states; i++) {
        this->cur_trajectory.at(i).pose.deepCopy(this->cur_trajectory.at(i - 1).pose.manifoldPlus(
          (this->param.scan_period / (this->param.num_trajectory_states - 1)) * this->previous_twist));
    }
}

void LaserOdom::buildTrees() {
    size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);

    // First step is to transform still active map features into the next frame
    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
        for (uint32_t j = 0; j < this->prv_feature_points.at(i).points.size(); j++) {
            if (this->feature_association.at(i).at(j).first > 0) {
                Vec3 transformed_pt;
                Eigen::Map<const Vec3> Vecpt(this->prv_feature_points.at(i).points.at(j).data(), 3, 1);
                transformed_pt = this->cur_trajectory.back().pose.inverseTransform(Vecpt);
                // Check if feature is within range of map
                if (l2length(transformed_pt.data(), 3) < this->param.local_map_range) {
                    memcpy(this->prv_feature_points.at(i).points.at(j).data(), transformed_pt.data(), 24);
                    if (this->feature_association.at(i).at(j).second == AssociationStatus::CORRESPONDED) {
                        this->feature_association.at(i).at(j).second = AssociationStatus::UNCORRESPONDED;
                        this->feature_association.at(i).at(j).first = this->param.TTL;
                    } else {
                        --(this->feature_association.at(i).at(j).first);
                    }
                    continue;
                }
            }
            auto loc = this->prv_feature_points.at(i).points.size() - 1;
            memcpy(this->prv_feature_points.at(i).points.at(j).data(),
                   this->prv_feature_points.at(i).points.at(loc).data(),
                   24);
            this->feature_association.at(i).at(j) = this->feature_association.at(i).at(loc);
            this->prv_feature_points.at(i).points.resize(loc);
            this->feature_association.at(i).resize(loc);
        }
        // rebuild kdtree index
        if (this->prv_feature_points.at(i).points.size() > 0) {
            this->feature_idx.at(i)->buildIndex();
        }
        for (uint16_t j = 0; j < this->param.n_ring; j++) {
            for (PointXYZIT pt : this->feature_points.at(i).at(j)) {
                double transformed_pt[3] = {0};
                this->transformToEnd(pt.pt, pt.tick, transformed_pt);

                resultSet.init(&ret_index, &out_dist_sqr);

                this->feature_idx.at(i)->findNeighbors(resultSet, transformed_pt, nanoflann::SearchParams(32, 1));
                if (out_dist_sqr > this->param.map_density) {
                    this->feature_association.at(i).push_back(
                      std::make_pair(this->param.TTL, AssociationStatus::UNCORRESPONDED));
                    this->prv_feature_points.at(i).points.push_back(
                      std::array<double, 3>{transformed_pt[0], transformed_pt[1], transformed_pt[2]});
                }
            }
        }
        // rebuild kdtree index
        if (this->prv_feature_points.at(i).points.size() > 0) {
            this->feature_idx.at(i)->buildIndex();
        }
        LOG_INFO("There are %lu feature type %u in the local map", this->prv_feature_points.at(i).points.size(), i);
    }
}

bool LaserOdom::findCorrespondingPoints(const Vec3 &query, const uint32_t &f_idx, std::vector<size_t> *index) {
    using ContType = std::pair<size_t, double>;
    std::vector<ContType> indices_dists;
    nanoflann::RadiusResultSet<double, size_t> resultSet(this->param.max_correspondence_dist, indices_dists);
    uint32_t knn = 0;
    switch (this->feature_definitions.at(f_idx).residual) {
        case PointToLine: knn = 2; break;
        case PointToPlane: knn = 3; break;
        default: knn = 0;
    }
    this->feature_idx.at(f_idx)->findNeighbors(resultSet, query.data(), nanoflann::SearchParams());
    if (indices_dists.size() < knn) {
        return false;
    }
    std::sort(indices_dists.begin(), indices_dists.end(), [](const ContType &left, const ContType &right) {
        return left.second < right.second;
    });

    index->clear();
    // In order to ensure correspondences are not picked along a scan line,
    // use bins and enforce that the set of correspondences should fall into at
    // least 2 bins
    double offset = 0;
    double current_azimuth = 0;
    uint16_t counter = 0;
    bool non_zero_bin = false;
    double const *point;
    while (counter < indices_dists.size()) {
        point = this->prv_feature_points.at(f_idx).points.at(indices_dists.at(counter).first).data();
        if (counter == 0) {
            offset = std::atan2(point[2], std::sqrt(point[0] * point[0] + point[1] * point[1]));
        } else {
            current_azimuth = std::atan2(point[2], std::sqrt(point[0] * point[0] + point[1] * point[1]));
            double t_bin = (current_azimuth - offset) / this->param.azimuth_tol;
            int cur_bin = t_bin > 0 ? (int) (t_bin + 0.5) : (int) (t_bin - 0.5);
            if (cur_bin != 0) {
                non_zero_bin = true;
            }
        }
        if (index->size() + 1 != knn || non_zero_bin) {
            index->push_back(indices_dists.at(counter).first);
        }
        if (index->size() == knn) {
            return true;
        }
        ++counter;
    }
    return false;
}

bool LaserOdom::match() {
    double zero_pt[3] = {0};

    ceres::Problem problem;

    std::vector<size_t> ret_indices;
    std::vector<double> out_dist_sqr;

    // set up pointer for evaluating residuals
    const double **parameters;
    parameters = new const double *[4];

    ceres::LocalParameterization *se3_param = new NullSE3Parameterization;
    std::vector<const ceres::LocalParameterization *> local_param_vec;

    // Add motion residuals
    // The prior for the pose is identity, the prior for twist is the twist at the end of the previous
    // trajectory
    this->cv_vector.at(0).calculateLinInvCovariance();

    Transformation<> identity;
    ceres::CostFunction *prior_cost =
      new TrajectoryPrior(this->cv_vector.at(0).inv_covar.sqrt(), identity, this->previous_twist);

    problem.AddResidualBlock(prior_cost,
                             nullptr,
                             this->cur_trajectory.at(0).pose.getInternalMatrix().derived().data(),
                             this->cur_trajectory.at(0).twist.data());

    for (uint32_t i = 0; i < this->param.num_trajectory_states; i++) {
        local_param_vec.emplace_back(se3_param);
        problem.AddParameterBlock(this->cur_trajectory.at(i).pose.getInternalMatrix().derived().data(), 12, se3_param);
        problem.AddParameterBlock(this->cur_trajectory.at(i).twist.data(), 6);

        if (i + 1 < this->param.num_trajectory_states) {
            this->cv_vector.at(i).calculateLinInvCovariance();

            ceres::CostFunction *motion_cost = new ConstantVelocityPrior(
              this->cv_vector.at(i).inv_covar.sqrt(), this->cv_vector.at(i).tkp1 - this->cv_vector.at(i).tk);
            problem.AddResidualBlock(motion_cost,
                                     nullptr,
                                     this->cur_trajectory.at(i).pose.getInternalMatrix().derived().data(),
                                     this->cur_trajectory.at(i + 1).pose.getInternalMatrix().derived().data(),
                                     this->cur_trajectory.at(i).twist.data(),
                                     this->cur_trajectory.at(i + 1).twist.data());
        }
    }

    // now loop over each type of feature, and generate residuals for each
    Eigen::Matrix<double, 12, 12> candle, hat;

    uint32_t k, kp1;
    double tau;

    for (uint16_t i = 0; i < this->N_FEATURES; i++) {
        for (uint16_t j = 0; j < this->param.n_ring; j++) {
            this->feature_corrs.at(i).at(j).clear();
            for (uint64_t pt_cntr = 0; pt_cntr < this->feature_points.at(i).at(j).size(); pt_cntr++) {
                double transformed[3];
                this->transformToStart(this->feature_points.at(i).at(j).at(pt_cntr).pt,
                                       this->feature_points.at(i).at(j).at(pt_cntr).tick,
                                       transformed,
                                       k,
                                       kp1,
                                       tau);
                Eigen::Map<const Vec3> query(transformed, 3, 1);
                ret_indices.clear();
                out_dist_sqr.clear();
                Eigen::Matrix3f covZ;
                this->range_sensor->getEuclideanCovariance(query.data(), j, covZ);
                if (this->findCorrespondingPoints(query, i, &ret_indices)) {
                    this->cv_vector.at(k).tau = &tau;
                    this->cv_vector.at(k).calculateStuff(hat, candle);
                    ceres::CostFunction *cost_function;
                    std::vector<double> residuals;
                    switch (this->feature_definitions.at(i).residual) {
                        case PointToLine:
                            if (this->param.treat_lines_as_planes) {
                                cost_function = new SE3PointToPlaneGP(
                                  this->feature_points.at(i).at(j).at(pt_cntr).pt,
                                  this->prv_feature_points.at(i).points.at(ret_indices.at(0)).data(),
                                  this->prv_feature_points.at(i).points.at(ret_indices.at(1)).data(),
                                  zero_pt,
                                  hat.block<6, 12>(0, 0),
                                  candle.block<6, 12>(0, 0),
                                  covZ.cast<double>(),
                                  this->param.use_weighting);
                                residuals.resize(1);
                            } else {
                                cost_function = new SE3PointToLineGP(
                                  this->feature_points.at(i).at(j).at(pt_cntr).pt,
                                  this->prv_feature_points.at(i).points.at(ret_indices.at(0)).data(),
                                  this->prv_feature_points.at(i).points.at(ret_indices.at(1)).data(),
                                  hat.block<6, 12>(0, 0),
                                  candle.block<6, 12>(0, 0),
                                  covZ.cast<double>(),
                                  this->param.use_weighting);
                                residuals.resize(2);
                            }
                            break;
                        case PointToPlane:
                            cost_function =
                              new SE3PointToPlaneGP(this->feature_points.at(i).at(j).at(pt_cntr).pt,
                                                    this->prv_feature_points.at(i).points.at(ret_indices.at(0)).data(),
                                                    this->prv_feature_points.at(i).points.at(ret_indices.at(1)).data(),
                                                    this->prv_feature_points.at(i).points.at(ret_indices.at(2)).data(),
                                                    hat.block<6, 12>(0, 0),
                                                    candle.block<6, 12>(0, 0),
                                                    covZ.cast<double>(),
                                                    this->param.use_weighting);
                            residuals.resize(1);
                            break;
                        default: continue;
                    }

                    parameters[0] = this->cur_trajectory.at(k).pose.getInternalMatrix().derived().data();
                    parameters[1] = this->cur_trajectory.at(kp1).pose.getInternalMatrix().derived().data();
                    parameters[2] = this->cur_trajectory.at(k).twist.data();
                    parameters[3] = this->cur_trajectory.at(kp1).twist.data();

                    if (!cost_function->Evaluate(parameters, residuals.data(), nullptr)) {
                        LOG_ERROR("Cost function did not evaluate");
                        delete cost_function;
                        continue;
                    }
                    if (norm(residuals) > this->param.max_residual_val) {
                        delete cost_function;
                        continue;
                    }
                    //                    if (this->param.check_gradients) {
                    //                        ceres::NumericDiffOptions ndiff_options;
                    //                        ceres::GradientChecker g_check(cost_function, &local_param_vec,
                    //                        ndiff_options);
                    //                        ceres::GradientChecker::ProbeResults g_results;
                    //                        if (!g_check.Probe(parameters, 1, &g_results)) {
                    //                            LOG_ERROR("%s", g_results.error_log.c_str());
                    //                        }
                    //                    }
                    std::vector<uint64_t> corr_list;
                    corr_list.emplace_back(pt_cntr);
                    for (uint32_t k = 0; k < ret_indices.size(); k++) {
                        corr_list.emplace_back(ret_indices.at(k));
                        this->feature_association.at(i).at(ret_indices.at(k)).second = AssociationStatus::CORRESPONDED;
                    }
                    this->feature_corrs.at(i).at(j).emplace_back(corr_list);
                    ceres::LossFunction *p_LossFunction = new BisquareLoss(this->param.robust_param);
                    problem.AddResidualBlock(cost_function,
                                             p_LossFunction,
                                             this->cur_trajectory.at(k).pose.getInternalMatrix().derived().data(),
                                             this->cur_trajectory.at(kp1).pose.getInternalMatrix().derived().data(),
                                             this->cur_trajectory.at(k).twist.data(),
                                             this->cur_trajectory.at(kp1).twist.data());
                }
            }
        }
    }

    ceres::Solver::Options options;  // ceres problem destructor apparently destructs quite a bit, so need to
                                     // instantiate
    // options struct every time :(
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 300;
    options.function_tolerance = 1e-10;
    options.parameter_tolerance = 1e-6;
    options.num_threads = std::thread::hardware_concurrency();
    options.num_linear_solver_threads = std::thread::hardware_concurrency();
    options.max_num_consecutive_invalid_steps = 2;

    ceres::Covariance::Options covar_options;
    covar_options.num_threads = std::thread::hardware_concurrency();
    covar_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
    covar_options.algorithm_type = ceres::CovarianceAlgorithmType::SPARSE_QR;

//    problem.SetParameterBlockConstant(this->cur_trajectory.at(0).pose.getInternalMatrix().derived().data());

    if (problem.NumResidualBlocks() < this->param.min_residuals) {
        LOG_ERROR("Less than expected residuals, resetting");
        LOG_ERROR("%d residuals, threshold is %d", problem.NumResidualBlocks(), this->param.min_residuals);
        this->resetTrajectory();
        this->initialized = false;
        return false;
    } else if (!this->param.only_extract_features) {
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
//        LOG_INFO("%s", summary.FullReport().c_str());
        //        ceres::Covariance covariance(covar_options);
        //        covariance.Compute(this->covariance_blocks, &problem);
        //        covariance.GetCovarianceBlock(
        //          this->cur_transform.getInternalMatrix().derived().data(),
        //          this->cur_transform.getInternalMatrix().derived().data(),
        //          this->covar);
    }
    return true;
}

void LaserOdom::resetTrajectory() {
    for (uint32_t i = 0; i < this->cur_trajectory.size(); i++) {
        this->cur_trajectory.at(i).pose.setIdentity();
        this->cur_trajectory.at(i).twist.setZero();
        this->previous_twist.setZero();
    }
}

/*
 * Alright, so first sort the filtered scores for each signal
 * Then go through each feature definition and pick out features
 */
void LaserOdom::generateFeatures() {
    this->computeScores();
    this->prefilter();

    for (unlong j = 0; j < this->param.n_ring; j++) {
        for (uint32_t i = 0; i < this->N_FEATURES; i++) {
            auto &def = this->feature_definitions.at(i);
            auto &pol = def.criteria.at(0).sel_pol;
            auto &filt_scores = this->filtered_scores.at(i).at(j);
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
            this->feature_points.at(i).at(j).clear();
            int f_cnt = 0;
            for (auto iter = filt_scores.begin(); iter != filt_scores.end(); iter++) {
                if (f_cnt >= *(def.n_limit)) {
                    break;
                }
                if (this->valid_pts.at(i).at(j).at(iter->first)) {
                    this->feature_points.at(i).at(j).emplace_back(this->cur_scan.at(j).at(iter->first).x,
                                                                  this->cur_scan.at(j).at(iter->first).y,
                                                                  this->cur_scan.at(j).at(iter->first).z,
                                                                  this->cur_scan.at(j).at(iter->first).intensity,
                                                                  this->cur_scan.at(j).at(iter->first).tick);
                    this->flagNearbyPoints(i, j, iter->first);
                    f_cnt++;
                }
            }
        }
    }
}

void LaserOdom::computeScores() {
    Eigen::array<ptrdiff_t, 1> dims({0});
    for (unlong i = 0; i < this->param.n_ring; i++) {
        // todo(ben) TensorMaps seem broken. Research further at some point
        for (ulong j = 0; j < this->N_SCORES; j++) {
            // todo(ben) Include signal to score explicitly in feature definitions and get
            // rid of this hack.
            ulong s_idx = 0;
            if (j < 1) {
                s_idx = 0;
            } else {
                s_idx = 1;
            }
            Eigen::Tensor<double, 1> inputmap((int) this->signals.at(s_idx).at(i).size());
            memcpy(inputmap.data(), this->signals.at(s_idx).at(i).data(), 8 * this->signals.at(s_idx).at(i).size());
            // Eigen::TensorMap<double, 1> inputmap(this->signals.at(j).at(i).data(),
            // (int)this->signals.at(j).at(i).size());
            if (inputmap.size() + 1 > kernels.at(j)->size()) {
                this->scores.at(j).at(i).resize(inputmap.size() - kernels.at(j)->size() + 1);
                Eigen::Tensor<double, 1> outputmap((int) inputmap.size() - kernels.at(j)->size() + 1);
                // Eigen::TensorMap<double, 1> outputmap(this->scores.at(j).at(i).data(),
                // this->scores.at(j).at(i).size());
                outputmap = inputmap.convolve(*(this->kernels.at(j)), dims);
                memcpy(this->scores.at(j).at(i).data(), outputmap.data(), 8 * outputmap.size());
            }
        }
    }
}

// This part filters out points that will not provide salient features
// based on range signal and any gaps.
void LaserOdom::prefilter() {
    for (uint32_t i = 0; i < this->param.n_ring; i++) {
        // The scores will be offset from the points by (kernel_size - 1)/2
        std::vector<bool> valid(this->signals[0].at(i).size(), true);
        // Now loop through the points in this ring and set valid_idx accordingly
        for (unlong j = 1; j + 1 < this->cur_scan.at(i).size(); j++) {
            auto delforward = this->l2sqrd(this->cur_scan.at(i).at(j), this->cur_scan.at(i).at(j + 1));
            auto delback = this->l2sqrd(this->cur_scan.at(i).at(j), this->cur_scan.at(i).at(j - 1));
            // First section excludes any points who's score is likely caused
            // by occlusion
            if (delforward > this->param.occlusion_tol_2) {
                double &d1 = this->signals[0].at(i).at(j);
                double &d2 = this->signals[0].at(i).at(j + 1);
                auto unit1 = this->scale(this->cur_scan.at(i).at(j), 1.0 / d1);
                auto unit2 = this->scale(this->cur_scan.at(i).at(j + 1), 1.0 / d2);
                auto diff = std::sqrt(this->l2sqrd(unit1, unit2));
                if (diff < this->param.occlusion_tol) {
                    // todo(ben) replace magic 5 with something
                    if (d1 > d2) {
                        for (unlong l = 0; l <= 5; l++) {
                            if (j >= l) {
                                valid.at(j - l) = false;
                            }
                        }
                    } else {
                        for (unlong l = 1; l <= 5; l++) {
                            if (j + l < this->signals[0].at(i).size()) {
                                valid.at(j + l) = false;
                            }
                        }
                    }
                }
            }
            // This section excludes any points whose nearby surface is
            // near to parallel to the laser beam
            auto dis = this->l2sqrd(this->cur_scan.at(i).at(j));
            if ((delforward > (this->param.parallel_tol) * dis) && (delback > (this->param.parallel_tol * dis))) {
                valid.at(j) = false;
            }
        }
        // now store each selected point for sorting
        // Each point will be only be put in if it is a valid candidate for
        // that feature type
        for (uint32_t k = 0; k < this->N_FEATURES; k++) {
            this->buildFilteredScore(valid, k, i);
        }
    }
}

bool near_zero_score(double score, double threshold) {
    return std::abs(score) < threshold;
}

bool high_pos_score(double score, double threshold) {
    return score > threshold;
}

bool high_neg_score(double score, double threshold) {
    return score < -threshold;
}

bool null_score(double score, double threshold) {
    return false;
}

void LaserOdom::buildFilteredScore(const std::vector<bool> &valid, const uint32_t &f_idx, const uint32_t &ring) {
    this->filtered_scores.at(f_idx).at(ring).clear();
    this->valid_pts.at(f_idx).at(ring).clear();
    auto &def = this->feature_definitions.at(f_idx);
    // get primary score index by kernel type
    std::vector<std::function<bool(double, double)>> compfuns;
    std::vector<uint32_t> k_idx;
    std::vector<uint32_t> k_offsets;
    uint32_t offset = 0;
    compfuns.resize(def.criteria.size());
    k_idx.resize(def.criteria.size());
    for (uint32_t i = 0; i < def.criteria.size(); i++) {
        switch (def.criteria.at(i).sel_pol) {
            case SelectionPolicy::NEAR_ZERO: compfuns.at(i) = near_zero_score; break;
            case SelectionPolicy::HIGH_POS: compfuns.at(i) = high_pos_score; break;
            case SelectionPolicy::HIGH_NEG: compfuns.at(i) = high_neg_score; break;
            default: compfuns.at(i) = null_score;
        }
        switch (def.criteria.at(i).kernel) {
            case Kernel::LOAM: k_idx.at(i) = 0; break;
            case Kernel::LOG: k_idx.at(i) = 1; break;
            case Kernel::FOG: k_idx.at(i) = 2; break;
            default: k_idx.at(i) = 0;
        }
        k_offsets.emplace_back((this->kernels.at(k_idx.at(i))->size() - 1) / 2);
        if (offset < (this->kernels.at(k_idx.at(i))->size() - 1) / 2) {
            offset = (this->kernels.at(k_idx.at(i))->size() - 1) / 2;
        }
    }

    // copy valid vector for use later
    this->valid_pts.at(f_idx).at(ring) = std::vector<bool>(valid.begin(), valid.end());

    for (uint64_t j = offset; j + offset < valid.size(); j++) {
        if (valid.at(j)) {
            bool still_good = true;
            for (uint32_t l = 0; l < def.criteria.size(); l++) {
                // todo(ben) Each score should have a different offset. But for now the offset between kernels is the
                // same
                // so it doesn't matter yet
                if (!(compfuns.at(l))(this->scores.at(k_idx.at(l)).at(ring).at(j - k_offsets.at(l)),
                                      *(def.criteria.at(l).threshold))) {
                    still_good = false;
                    break;
                }
            }
            if (still_good) {
                this->filtered_scores[f_idx].at(ring).emplace_back(j,
                                                                   this->scores[k_idx.at(0)].at(ring).at(j - offset));
            }
        }
    }
}

}  // namespace wave
