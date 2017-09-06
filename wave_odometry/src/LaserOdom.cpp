#include "wave/odometry/LaserOdom.hpp"

namespace wave {

double l2length(const double *const vec, uint16_t length) {
    double retval = 0;
    for (uint16_t i = 0; i < length; i++) {
        retval += vec[i] * vec[i];
    }
    return retval;
}

double getDiff(double A[], double B[], int length) {
    double retval = 0;
    for (int i = 0; i < length; i++) {
        retval += (A[i] - B[i]) * (A[i] - B[i]);
    }
    return std::sqrt(retval);
}

void LaserOdom::flagNearbyPoints(const unlong ring, const unlong index) {
    for (unlong j = 0; j < this->param.key_radius; j++) {
        if (index + j + 1 >= this->cur_scan.at(ring).size()) {
            break;
        }
        if (this->l2sqrd(this->cur_scan.at(ring).at(index + j), this->cur_scan.at(ring).at(index + j + 1)) >
            this->param.keypt_radius) {
            break;
        }
        this->cur_curve.at(ring).at(index + j + 1).first = false;
    }
    for (unlong j = 0; j < this->param.key_radius; j++) {
        if (index < j + 1) {
            break;
        }
        if (this->l2sqrd(this->cur_scan.at(ring).at(index - j), this->cur_scan.at(ring).at(index - j - 1)) >
            this->param.keypt_radius) {
            break;
        }
        this->cur_curve.at(ring).at(index - j - 1).first = false;
    }
}

void LaserOdom::transformToStart(const double *const pt, const uint16_t tick, double *output) {
    double &scale = this->scale_lookup.at(tick);
    double angle_axis[3] = {
      scale * this->undistort_rotation[0], scale * this->undistort_rotation[1], scale * this->undistort_rotation[2]};
    ceres::AngleAxisRotatePoint(angle_axis, pt, output);
    output[0] += scale * this->undistort_translation[0];
    output[1] += scale * this->undistort_translation[1];
    output[2] += scale * this->undistort_translation[2];
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

LaserOdom::LaserOdom(const LaserOdomParams params) : param(params) {
    auto n_ring = static_cast<size_t>(param.n_ring);
    this->cur_scan.resize(n_ring);
    this->cur_curve.resize(n_ring);
    this->filter.resize(n_ring);
    this->flats.resize(n_ring);
    this->edges.resize(n_ring);
    this->edge_idx = new kd_tree_t(3, this->prv_edges, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    this->flat_idx = new kd_tree_t(3, this->prv_flats, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    this->cur_translation = {0, 0, 0};
    this->cur_rotation = {0, 0, 0};
    this->prev_translation = {0, 0, 0};
    this->prev_rotation = {0, 0, 0};
    if (params.visualize) {
        this->display = new PointCloudDisplay("laser odom", 0.05);
        this->display->startSpin();
        // Allocate space for visualization clouds
        this->prev_viz = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>);
        this->cur_viz = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>);
    }
    this->scale_lookup.resize(this->param.max_ticks + 1, 0);  // plus one is for 0 tick
    for (int i = 0; i <= this->param.max_ticks; i++) {
        this->scale_lookup.at(i) = (double) i / (double) this->param.max_ticks;
    }
    if (params.output_trajectory) {
        long timestamp = std::chrono::system_clock::now().time_since_epoch().count();
        this->file.open(std::to_string(timestamp) + "laser_odom_traj.txt");
    }

    // ceres optimizer options
    std::vector<int> iterations = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    this->ceres_options.linear_solver_type = ceres::DENSE_QR;
    this->ceres_options.max_num_iterations = 300;
    this->ceres_options.function_tolerance = 1e-10;
    this->ceres_options.parameter_tolerance = 1e-6;
    this->ceres_options.num_threads = 4;
    this->ceres_options.num_linear_solver_threads = 4;
    // this->ceres_options.trust_region_minimizer_iterations_to_dump = iterations;
    this->ceres_options.trust_region_problem_dump_format_type = ceres::DumpFormatType::TEXTFILE;

    // ceres covariance setup
    this->covar_options.num_threads = 4;
    this->covar_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
    this->covar_options.algorithm_type = ceres::CovarianceAlgorithmType::SPARSE_QR;
    this->covariance_blocks.push_back(std::make_pair(this->cur_rotation.data(), this->cur_rotation.data()));
    this->covariance_blocks.push_back(std::make_pair(this->cur_rotation.data(), this->cur_translation.data()));
    this->covariance_blocks.push_back(std::make_pair(this->cur_translation.data(), this->cur_translation.data()));
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
    delete this->edge_idx;
    delete this->flat_idx;
    if (this->output_thread) {
        this->continue_output = false;
        this->output_condition.notify_one();
        this->output_thread->join();
    }
    this->cur_scan.clear();
    this->cur_curve.clear();
    this->filter.clear();
    this->edges.clear();
    this->flats.clear();
}

void LaserOdom::registerOutputFunction(
  std::function<void(const TimeType *const,
                     const std::array<double, 3> *const,
                     const std::array<double, 3> *const,
                     const pcl::PointCloud<pcl::PointXYZI> *const)> output_function) {
    this->f_output = std::bind(output_function,
                               &(this->undistorted_stamp),
                               &(this->undistort_rotation),
                               &(this->undistort_translation),
                               &(this->undistorted_cld));
    this->output_thread = std::unique_ptr<std::thread>(new std::thread(&LaserOdom::spinOutput, this));
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
    this->undis_edges.clear();
    this->undis_flats.clear();
    this->map_edges.resize(this->prv_edges.points.size());
    this->map_flats.resize(this->prv_flats.points.size());
    this->edge_cor.clear();
    this->flat_cor.clear();
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
        for (uint32_t i = 0; i < this->edges.at(r_idx).size(); i++) {
            double point[3] = {
              this->edges.at(r_idx).at(i).pt[0], this->edges.at(r_idx).at(i).pt[1], this->edges.at(r_idx).at(i).pt[2]};
            double u_pt[3];
            this->transformToStart(point, this->edges.at(r_idx).at(i).tick, u_pt);
            pcl::PointXYZ op_pt;
            op_pt.x = (float) u_pt[0];
            op_pt.y = (float) u_pt[1];
            op_pt.z = (float) u_pt[2];
            this->undis_edges.push_back(op_pt);
        }
        for (uint32_t i = 0; i < this->flats.at(r_idx).size(); i++) {
            double point[3] = {
              this->flats.at(r_idx).at(i).pt[0], this->flats.at(r_idx).at(i).pt[1], this->flats.at(r_idx).at(i).pt[2]};
            double u_pt[3];
            this->transformToStart(point, this->flats.at(r_idx).at(i).tick, u_pt);
            pcl::PointXYZ op_pt;
            op_pt.x = (float) u_pt[0];
            op_pt.y = (float) u_pt[1];
            op_pt.z = (float) u_pt[2];
            this->undis_flats.push_back(op_pt);
        }
    }
    for (size_t i = 0; i < this->prv_edges.points.size(); i++) {
        // stupid friggin pcl and its floats
        this->map_edges.points.at(i).x = this->prv_edges.points.at(i).at(0);
        this->map_edges.points.at(i).y = this->prv_edges.points.at(i).at(1);
        this->map_edges.points.at(i).z = this->prv_edges.points.at(i).at(2);
    }
    for (size_t i = 0; i < this->prv_flats.points.size(); i++) {
        // stupid friggin pcl and its floats
        this->map_flats.points.at(i).x = this->prv_flats.points.at(i).at(0);
        this->map_flats.points.at(i).y = this->prv_flats.points.at(i).at(1);
        this->map_flats.points.at(i).z = this->prv_flats.points.at(i).at(2);
    }

    for (auto iter = this->edge_corrs.begin(); iter != this->edge_corrs.end(); iter++) {
        std::array<double, 12> new_corr;
        double undis[3];
        this->transformToStart(&(this->edges.at(iter->at(0)).at(iter->at(1)).pt[0]),
                               this->edges.at(iter->at(0)).at(iter->at(1)).tick,
                               undis);
        new_corr[0] = this->edges.at(iter->at(0)).at(iter->at(1)).pt[0];
        new_corr[1] = this->edges.at(iter->at(0)).at(iter->at(1)).pt[1];
        new_corr[2] = this->edges.at(iter->at(0)).at(iter->at(1)).pt[2];
        new_corr[3] = this->prv_edges.points.at(iter->at(2)).at(0);
        new_corr[4] = this->prv_edges.points.at(iter->at(2)).at(1);
        new_corr[5] = this->prv_edges.points.at(iter->at(2)).at(2);
        new_corr[6] = this->prv_edges.points.at(iter->at(3)).at(0);
        new_corr[7] = this->prv_edges.points.at(iter->at(3)).at(1);
        new_corr[8] = this->prv_edges.points.at(iter->at(3)).at(2);
        new_corr[9] = undis[0];
        new_corr[10] = undis[1];
        new_corr[11] = undis[2];
        this->edge_cor.push_back(new_corr);
    }
    for (auto iter = this->flat_corrs.begin(); iter != this->flat_corrs.end(); iter++) {
        std::array<double, 15> new_corr;
        double undis[3];
        this->transformToStart(&(this->flats.at(iter->at(0)).at(iter->at(1)).pt[0]),
                               this->flats.at(iter->at(0)).at(iter->at(1)).tick,
                               undis);

        new_corr[0] = this->flats.at(iter->at(0)).at(iter->at(1)).pt[0];
        new_corr[1] = this->flats.at(iter->at(0)).at(iter->at(1)).pt[1];
        new_corr[2] = this->flats.at(iter->at(0)).at(iter->at(1)).pt[2];
        new_corr[3] = this->prv_flats.points.at(iter->at(2)).at(0);
        new_corr[4] = this->prv_flats.points.at(iter->at(2)).at(1);
        new_corr[5] = this->prv_flats.points.at(iter->at(2)).at(2);
        new_corr[6] = this->prv_flats.points.at(iter->at(3)).at(0);
        new_corr[7] = this->prv_flats.points.at(iter->at(3)).at(1);
        new_corr[8] = this->prv_flats.points.at(iter->at(3)).at(2);
        new_corr[9] = this->prv_flats.points.at(iter->at(4)).at(0);
        new_corr[10] = this->prv_flats.points.at(iter->at(4)).at(1);
        new_corr[11] = this->prv_flats.points.at(iter->at(4)).at(2);
        new_corr[12] = undis[0];
        new_corr[13] = undis[1];
        new_corr[14] = undis[2];
        this->flat_cor.push_back(new_corr);
    }
}

void LaserOdom::addPoints(const std::vector<PointXYZIR> &pts, const int tick, TimeType stamp) {
    if ((tick - this->prv_tick) < -200) {  // tolerate minor nonlinearity error
        this->generateFeatures();          // generate features on the current scan
        if (this->initialized) {           // there is a set of previous features from
                                           // last scan
            double last_translation[3], last_rotation[3];
            ceres::Problem problem;
            for (int i = 0; i < this->param.opt_iters; i++) {
                if (i > 0) {
                    memcpy(last_translation, this->cur_translation.data(), 24);
                    memcpy(last_rotation, this->cur_rotation.data(), 24);
                }
                if (!this->match(&problem)) {
                    return;
                }
                if (i > 0) {
                    double diff = getDiff(last_translation, this->cur_translation.data(), 3) +
                                  getDiff(last_rotation, this->cur_rotation.data(), 3);
                    if (diff < this->param.diff_tol) {
                        break;
                    }
                }
            }

            ceres::Covariance covariance(this->covar_options);
            covariance.Compute(this->covariance_blocks, &problem);
            covariance.GetCovarianceBlock(this->cur_rotation.data(), this->cur_rotation.data(), this->covar_ww);
            covariance.GetCovarianceBlock(
                    this->cur_rotation.data(), this->cur_translation.data(), this->covar_wt);
            covariance.GetCovarianceBlock(
                    this->cur_translation.data(), this->cur_translation.data(), this->covar_tt);

            memcpy(this->prev_translation.data(), this->cur_translation.data(), 24);
            memcpy(this->prev_rotation.data(), this->cur_rotation.data(), 24);
            if (this->param.output_trajectory) {
                this->file << this->cur_rotation[0] << ", " << this->cur_rotation[1] << ", " << this->cur_rotation[2]
                           << ", " << this->cur_translation[0] << ", " << this->cur_translation[1] << ", "
                           << this->cur_translation[2] << std::endl;
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
                    memcpy(this->undistort_translation.data(), this->cur_translation.data(), 24);
                    memcpy(this->undistort_rotation.data(), this->cur_rotation.data(), 24);

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
        p.tick = tick;
        this->cur_scan.at(pt.ring).push_back(this->applyIMU(p));
    }
    this->prv_tick = tick;
}

void LaserOdom::updateViz() {
    // populate prev with points stored in kd tree
    // These are already transformed to the start of the current scan
    this->display->removeAll();
    this->prev_viz->clear();
    this->cur_viz->clear();

    for (auto iter = this->prv_flats.points.begin(); iter != this->prv_flats.points.end(); iter++) {
        pcl::PointXYZI pt;
        pt.x = (*iter).at(0);
        pt.y = (*iter).at(1);
        pt.z = (*iter).at(2);
        pt.intensity = 1;
        this->prev_viz->push_back(pt);
    }
    for (auto iter = this->prv_edges.points.begin(); iter != this->prv_edges.points.end(); iter++) {
        pcl::PointXYZI pt;
        pt.x = (*iter).at(0);
        pt.y = (*iter).at(1);
        pt.z = (*iter).at(2);
        pt.intensity = 1;
        this->prev_viz->push_back(pt);
    }

    // For the current features, need to transform them to start of scan
    std::vector<size_t> ret_indices(3);
    std::vector<double> out_dist_sqr(3);

    for (uint16_t i = 0; i < this->param.n_ring; i++) {
        for (auto iter = this->edges.at(i).begin(); iter != this->edges.at(i).end(); iter++) {
            double pt[3];
            double &scale = this->scale_lookup.at(iter->tick);
            double wvec[3] = {
              scale * this->cur_rotation[0], scale * this->cur_rotation[1], scale * this->cur_rotation[2]};
            ceres::AngleAxisRotatePoint(wvec, iter->pt, pt);
            pt[0] += scale * this->cur_translation[0];
            pt[1] += scale * this->cur_translation[1];
            pt[2] += scale * this->cur_translation[2];

            pcl::PointXYZI point;
            point.x = pt[0];
            point.y = pt[1];
            point.z = pt[2];
            point.intensity = 2;
            this->prev_viz->push_back(point);
        }
        for (auto iter = this->flats.at(i).begin(); iter != this->flats.at(i).end(); iter++) {
            double pt[3];
            double &scale = this->scale_lookup.at(iter->tick);
            double wvec[3] = {
              scale * this->cur_rotation[0], scale * this->cur_rotation[1], scale * this->cur_rotation[2]};
            ceres::AngleAxisRotatePoint(wvec, iter->pt, pt);
            pt[0] += scale * this->cur_translation[0];
            pt[1] += scale * this->cur_translation[1];
            pt[2] += scale * this->cur_translation[2];

            pcl::PointXYZI point;
            point.x = pt[0];
            point.y = pt[1];
            point.z = pt[2];
            point.intensity = 2;
            this->prev_viz->push_back(point);
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
        this->cur_curve.at(i).clear();
        this->cur_scan.at(i).clear();
    }
    if (!this->initialized) {
        size_t cnt_edges = this->prv_edges.points.size();
        size_t cnt_flats = this->prv_flats.points.size();

        if ((cnt_edges >= this->param.n_edge) && (cnt_flats >= this->param.n_flat)) {
            this->initialized = true;
        }
    }
}

void LaserOdom::buildTrees() {
    //    this->prv_edges.points.clear();
    //    this->prv_flats.points.clear();
    size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);

    double axis_angle_inv[3] = {-this->cur_rotation[0], -this->cur_rotation[1], -this->cur_rotation[2]};
    double trans[3] = {this->cur_translation[0], this->cur_translation[1], this->cur_translation[2]};
    double RinvT[3];
    ceres::AngleAxisRotatePoint(axis_angle_inv, trans, RinvT);

    // Transform existing edge-plane features to the lidar frame at the end of the current scan
    for (uint16_t i = 0; i < this->prv_edges.points.size(); i++) {
        double transformed_pt[3];
        ceres::AngleAxisRotatePoint(axis_angle_inv, this->prv_edges.points.at(i).data(), transformed_pt);
        transformed_pt[0] -= RinvT[0];
        transformed_pt[1] -= RinvT[1];
        transformed_pt[2] -= RinvT[2];
        // Check if feature has travelled past range of map
        if (l2length(transformed_pt, 3) < this->param.local_map_range) {
            memcpy(this->prv_edges.points.at(i).data(), &transformed_pt, 24);
        } else {
            memcpy(this->prv_edges.points.at(i).data(),
                   this->prv_edges.points.at(this->prv_edges.points.size() - 1).data(),
                   24);
            this->edge_idx->removePoint(this->prv_edges.points.size() - 1);
            this->prv_edges.points.resize(this->prv_edges.points.size() - 1);
        }
        this->edge_idx->removePoint(i);
    }
    for (uint16_t i = 0; i < this->prv_flats.points.size(); i++) {
        double transformed_pt[3];
        ceres::AngleAxisRotatePoint(axis_angle_inv, this->prv_flats.points.at(i).data(), transformed_pt);
        transformed_pt[0] -= RinvT[0];
        transformed_pt[1] -= RinvT[1];
        transformed_pt[2] -= RinvT[2];
        // Check if feature has travelled beyond range of map
        if (l2length(transformed_pt, 3) < this->param.local_map_range) {
            memcpy(this->prv_flats.points.at(i).data(), &transformed_pt, 24);
        } else {
            memcpy(this->prv_flats.points.at(i).data(),
                   this->prv_flats.points.at(this->prv_flats.points.size() - 1).data(),
                   24);
            this->flat_idx->removePoint(this->prv_flats.points.size() - 1);
            this->prv_flats.points.resize(this->prv_flats.points.size() - 1);
        }
        this->flat_idx->removePoint(i);
    }
    // Rebuild kd trees
    if (this->prv_edges.points.size() > 0) {
        this->edge_idx->addPoints(0, this->prv_edges.points.size() - 1);
    }
    if (this->prv_flats.points.size() > 0) {
        this->flat_idx->addPoints(0, this->prv_flats.points.size() - 1);
    }

    for (uint16_t i = 0; i < this->param.n_ring; i++) {
        for (PointXYZIT pt : this->edges.at(i)) {
            double scale_rvs = 1 - this->scale_lookup.at(pt.tick);

            double axis_angle_inv_scaled[3] = {
              scale_rvs * axis_angle_inv[0], scale_rvs * axis_angle_inv[1], scale_rvs * axis_angle_inv[2]};
            double rotated_pt[3] = {0};
            ceres::AngleAxisRotatePoint(axis_angle_inv_scaled, pt.pt, rotated_pt);

            rotated_pt[0] -= scale_rvs * RinvT[0];
            rotated_pt[1] -= scale_rvs * RinvT[1];
            rotated_pt[2] -= scale_rvs * RinvT[2];

            resultSet.init(&ret_index, &out_dist_sqr);

            this->edge_idx->findNeighbors(resultSet, rotated_pt, nanoflann::SearchParams(32, 1));
            if (out_dist_sqr > this->param.map_density) {
                this->prv_edges.points.push_back(std::array<double, 3>{rotated_pt[0], rotated_pt[1], rotated_pt[2]});
                this->edge_idx->addPoints(this->prv_edges.points.size() - 1, this->prv_edges.points.size() - 1);
            }
        }

        for (PointXYZIT pt : this->flats.at(i)) {
            double scale_rvs = 1 - this->scale_lookup.at(pt.tick);

            double axis_angle_inv_scaled[3] = {
              scale_rvs * axis_angle_inv[0], scale_rvs * axis_angle_inv[1], scale_rvs * axis_angle_inv[2]};
            double rotated_pt[3] = {0};
            ceres::AngleAxisRotatePoint(axis_angle_inv_scaled, pt.pt, rotated_pt);

            rotated_pt[0] -= scale_rvs * RinvT[0];
            rotated_pt[1] -= scale_rvs * RinvT[1];
            rotated_pt[2] -= scale_rvs * RinvT[2];

            resultSet.init(&ret_index, &out_dist_sqr);

            this->flat_idx->findNeighbors(resultSet, rotated_pt, nanoflann::SearchParams(32, 1));
            if (out_dist_sqr > this->param.map_density) {
                this->prv_flats.points.push_back(std::array<double, 3>{rotated_pt[0], rotated_pt[1], rotated_pt[2]});
                this->flat_idx->addPoints(this->prv_flats.points.size() - 1, this->prv_flats.points.size() - 1);
            }
        }
    }
    LOG_INFO("There are %lu edges in the local map", this->prv_edges.points.size());
    LOG_INFO("There are %lu flats in the local map", this->prv_flats.points.size());
}

bool LaserOdom::findCorrespondingPoints(const Vec3 &query,
                                        const uint16_t knn,
                                        const bool searchPlanarPoints,
                                        std::vector<size_t> *index) {
    using ContType = std::pair<size_t, double>;
    std::vector<ContType> indices_dists;
    nanoflann::RadiusResultSet<double, size_t> resultSet(this->param.max_correspondence_dist, indices_dists);

    if (searchPlanarPoints) {
        this->flat_idx->findNeighbors(resultSet, query.data(), nanoflann::SearchParams());
    } else {
        this->edge_idx->findNeighbors(resultSet, query.data(), nanoflann::SearchParams());
    }
    if (indices_dists.size() < knn) {
        return false;
    }
    std::sort(indices_dists.begin(), indices_dists.end(), [](const ContType &left, const ContType &right) {
        return left.second < right.second;
    });
    // Now have sorted list of possible correpondences.
    // May want to revisit this later, but for now the policy will be
    // to pick the closest point + the remaining closest points that are not parallel to scan line.
    index->clear();
    double closest_azimuth = 0;
    double current_azimuth = 0;
    uint16_t counter = 0;
    double const * point;
    while (counter < indices_dists.size()) {
//        if (searchPlanarPoints) {
//            point = this->prv_flats.points.at(indices_dists.at(counter).first).data();
//        } else {
//            point = this->prv_edges.points.at(indices_dists.at(counter).first).data();
//        }
//        if (counter == 0) {
//            closest_azimuth = std::atan2(point[2] , std::sqrt(point[0] * point[0] + point[1]*point[1]));
//            index->push_back(indices_dists.at(counter).first);
//            ++counter;
//            continue;
//        }
//        current_azimuth = std::atan2(point[2] , std::sqrt(point[0] * point[0] + point[1]*point[1]));
//        if ((current_azimuth > closest_azimuth ? current_azimuth - closest_azimuth
//                                               : closest_azimuth - current_azimuth) > this->param.azimuth_tol) {
//            index->push_back(indices_dists.at(counter).first);
//        }
        index->push_back(indices_dists.at(counter).first);
        ++counter;
        if (index->size() == knn) {
            return true;
        }
    }

    return false;
}

bool LaserOdom::match(ceres::Problem *problem) {
    std::ofstream edge_cor, flat_cor;
    if (this->param.output_correspondences) {
        long timestamp = std::chrono::system_clock::now().time_since_epoch().count();
        edge_cor.open(std::to_string(timestamp) + "edges_cor.txt");
        flat_cor.open(std::to_string(timestamp) + "flat_cor.txt");
    }

    Vec3 trans(this->cur_translation[0], this->cur_translation[1], this->cur_translation[2]);

    std::vector<size_t> ret_indices;
    std::vector<double> out_dist_sqr;
    this->edge_corrs.clear();
    this->flat_corrs.clear();

    // set up pointer for evaluating residuals
    const double **parameters;
    parameters = new const double *[2];
    parameters[0] = this->cur_rotation.data();
    parameters[1] = this->cur_translation.data();

    // residual blocks for edges
    for (uint16_t i = 0; i < this->param.n_ring; i++) {
        for (size_t idx = 0; idx < this->edges.at(i).size(); idx++) {
            double &scale = this->scale_lookup.at(this->edges.at(i).at(idx).tick);

            Vec3 pt(this->edges.at(i).at(idx).pt[0], this->edges.at(i).at(idx).pt[1], this->edges.at(i).at(idx).pt[2]);
            double axis_angle[3] = {
              scale * this->cur_rotation[0], scale * this->cur_rotation[1], scale * this->cur_rotation[2]};
            double rotated[3];
            ceres::AngleAxisRotatePoint(axis_angle, pt.data(), rotated);
            Vec3 query(rotated[0], rotated[1], rotated[2]);
            query = query + scale * trans;
            if (this->findCorrespondingPoints(query, 2, false, &ret_indices)) {
                const double *refA = this->prv_edges.points.at(ret_indices.at(0)).data();
                const double *refB = this->prv_edges.points.at(ret_indices.at(1)).data();
                Vec3 point_A(refA[0], refA[1], refA[2]);
                Vec3 point_B(refB[0], refB[1], refB[2]);
                point_A.normalize();
                point_B.normalize();
                double approx_angle = (point_A - point_B).norm();
                if (approx_angle > this->param.max_line_dist) {
                    continue;
                }
                ceres::CostFunction *cost_function =
                  new AnalyticalPointToLine(&(this->edges.at(i).at(idx).pt[0]),
                                            this->prv_edges.points.at(ret_indices.at(0)).data(),
                                            this->prv_edges.points.at(ret_indices.at(1)).data(),
                                            &(this->scale_lookup.at(this->edges.at(i).at(idx).tick)));
                double residuals[3];
                double **jacobians = nullptr;
                if (!cost_function->Evaluate(parameters, residuals, jacobians)) {
                    LOG_ERROR("Point to line cost function did not evaluate");
                    continue;
                }
                if (l2length(residuals, 3) < this->param.max_correspondence_dist) {
                    std::array<uint16_t, 4> cur_corr;
                    cur_corr[0] = i;
                    cur_corr[1] = idx;
                    cur_corr[2] = ret_indices.at(0);
                    cur_corr[3] = ret_indices.at(1);
                    this->edge_corrs.push_back(cur_corr);
                    auto &pt_ref = this->edges.at(i).at(idx).pt;
                    auto &pA_ref = this->prv_edges.points.at(ret_indices.at(0));
                    auto &pB_ref = this->prv_edges.points.at(ret_indices.at(1));
                    if (this->param.output_correspondences) {
                        edge_cor << pt_ref[0] << ", " << pt_ref[1] << ", " << pt_ref[2] << ", " << pA_ref.at(0) << ", "
                                 << pA_ref.at(1) << ", " << pA_ref.at(2) << ", " << pB_ref.at(0) << ", " << pB_ref.at(1)
                                 << ", " << pB_ref.at(2) << ", " << scale << std::endl;
                    }
                    ceres::LossFunction *p_LossFunction = new BisquareLoss(this->param.huber_delta);
                    problem->AddResidualBlock(
                      cost_function, p_LossFunction, this->cur_rotation.data(), this->cur_translation.data());
                } else {
                    delete cost_function;
                }
            }
        }

        // residuals blocks for flats
        for (size_t ind = 0; ind < this->flats.at(i).size(); ind++) {
            double &scale = this->scale_lookup.at(this->flats.at(i).at(ind).tick);

            Vec3 pt(this->flats.at(i).at(ind).pt[0], this->flats.at(i).at(ind).pt[1], this->flats.at(i).at(ind).pt[2]);
            double axis_angle[3] = {
              scale * this->cur_rotation[0], scale * this->cur_rotation[1], scale * this->cur_rotation[2]};
            double rotated[3];
            ceres::AngleAxisRotatePoint(axis_angle, pt.data(), rotated);
            Vec3 query(rotated[0], rotated[1], rotated[2]);
            query = query + scale * trans;

            if (this->findCorrespondingPoints(query, 3, true, &ret_indices)) {
                ceres::CostFunction *cost_function =
                  new AnalyticalPointToPlane(&(this->flats.at(i).at(ind).pt[0]),
                                             this->prv_flats.points.at(ret_indices.at(0)).data(),
                                             this->prv_flats.points.at(ret_indices.at(1)).data(),
                                             this->prv_flats.points.at(ret_indices.at(2)).data(),
                                             &(this->scale_lookup.at(this->flats.at(i).at(ind).tick)));
                double residuals[1];
                double **jacobians = nullptr;
                if (!cost_function->Evaluate(parameters, residuals, jacobians)) {
                    LOG_ERROR("cost function did not evaluate");
                    continue;
                }
                if (l2length(residuals, 1) < this->param.max_correspondence_dist) {
                    std::array<uint16_t, 5> cur_corr;
                    cur_corr[0] = i;
                    cur_corr[1] = ind;
                    cur_corr[2] = ret_indices.at(0);
                    cur_corr[3] = ret_indices.at(1);
                    cur_corr[4] = ret_indices.at(2);
                    this->flat_corrs.push_back(cur_corr);

                    ceres::LossFunction *p_LossFunction = new BisquareLoss(this->param.huber_delta);
                    problem->AddResidualBlock(
                      cost_function, p_LossFunction, this->cur_rotation.data(), this->cur_translation.data());
                } else {
                    delete cost_function;
                }
            }
        }
    }
    if (this->param.output_correspondences) {
        edge_cor.close();
        flat_cor.close();
    }

    if (this->param.imposePrior) {
        ceres::Matrix rotation_stiffness(3, 3);
        ceres::Matrix translation_stiffness(3, 3);
        rotation_stiffness.setZero();
        translation_stiffness.setZero();
        rotation_stiffness(0, 0) =
          this->param.rotation_stiffness * this->param.RP_multiplier * this->param.decay_parameter;
        rotation_stiffness(1, 1) =
          this->param.rotation_stiffness * this->param.RP_multiplier * this->param.decay_parameter;
        rotation_stiffness(2, 2) = this->param.rotation_stiffness * this->param.decay_parameter;
        translation_stiffness(0, 0) = this->param.translation_stiffness * this->param.decay_parameter;
        translation_stiffness(1, 1) =
          this->param.translation_stiffness * this->param.T_y_multiplier * this->param.decay_parameter;
        translation_stiffness(2, 2) =
          this->param.translation_stiffness * this->param.T_z_multiplier * this->param.decay_parameter;
        ceres::VectorRef trans_ref(this->prev_translation.data(), 3, 1);
        ceres::VectorRef rot_ref(this->prev_rotation.data(), 3, 1);
        ceres::NormalPrior *t_diff_cost_function = new ceres::NormalPrior(translation_stiffness, trans_ref);
        ceres::NormalPrior *r_diff_cost_function = new ceres::NormalPrior(rotation_stiffness, rot_ref);
        problem->AddResidualBlock(t_diff_cost_function, NULL, this->cur_translation.data());
        problem->AddResidualBlock(r_diff_cost_function, NULL, this->cur_rotation.data());
    }

    delete[] parameters;

    if (problem->NumResidualBlocks() < this->param.min_residuals) {
        LOG_ERROR("Less than expected residuals, resetting");
        LOG_ERROR("%d residuals, threshold is %d", problem->NumResidualBlocks(), this->param.min_residuals);
        this->cur_translation = {0, 0, 0};
        this->cur_rotation = {0, 0, 0};
        this->prev_translation = {0, 0, 0};
        this->prev_rotation = {0, 0, 0};
        this->initialized = false;
        return false;
    } else if (!this->param.only_extract_features) {
        ceres::Solve(this->ceres_options, problem, &(this->ceres_summary));
        // LOG_INFO("%s", summary.FullReport().c_str());
    }
    return true;
}

/*
 * The gist of the features is too pick out high and low curvature scores.
 * However, to spread the features out, when selecting a feature invalidate
 * nearby points
 */
void LaserOdom::generateFeatures() {
    this->computeCurvature();
    this->prefilter();

    for (unlong i = 0; i < this->param.n_ring; i++) {
        this->edges.at(i).clear();
        this->flats.at(i).clear();
        std::sort(this->filter.at(i).begin(),
                  this->filter.at(i).end(),
                  [](const std::pair<unlong, float> lhs, const std::pair<unlong, float> rhs) {
                      return lhs.second < rhs.second;
                  });
        int edge_cnt = 0, flat_cnt = 0;

        // Pick out edge points
        for (auto iter = this->filter.at(i).rbegin(); iter < this->filter.at(i).rend(); iter++) {
            if ((iter->second < this->param.edge_tol) || (edge_cnt >= this->param.n_edge)) {
                break;
            } else if (this->cur_curve.at(i).at(iter->first).first) {
                this->edges.at(i).emplace_back(this->cur_scan.at(i).at(iter->first).x,
                                               this->cur_scan.at(i).at(iter->first).y,
                                               this->cur_scan.at(i).at(iter->first).z,
                                               this->cur_scan.at(i).at(iter->first).intensity,
                                               this->cur_scan.at(i).at(iter->first).tick);
                this->cur_curve.at(i).at(iter->first).first = false;
                this->flagNearbyPoints(i, iter->first);
                edge_cnt++;
            }
        }

        // Pick out flat points
        for (auto iter = this->filter.at(i).begin(); iter < this->filter.at(i).end(); iter++) {
            if ((iter->second > this->param.flat_tol) || (flat_cnt >= this->param.n_flat)) {
                break;
            } else if (this->cur_curve.at(i).at(iter->first).first) {
                this->flats.at(i).emplace_back(this->cur_scan.at(i).at(iter->first).x,
                                               this->cur_scan.at(i).at(iter->first).y,
                                               this->cur_scan.at(i).at(iter->first).z,
                                               this->cur_scan.at(i).at(iter->first).intensity,
                                               this->cur_scan.at(i).at(iter->first).tick);
                this->cur_curve.at(i).at(iter->first).first = false;
                this->flagNearbyPoints(i, iter->first);
                flat_cnt++;
            }
        }
    }
}

void LaserOdom::computeCurvature() {
    for (unlong i = 0; i < this->param.n_ring; i++) {
        auto n_pts = this->cur_scan.at(i).size();
        this->cur_curve.at(i).resize(n_pts);
        if (n_pts < this->param.knn + 1) {
            continue;
        }
        for (unlong j = this->param.knn; j < n_pts - this->param.knn; j++) {
            float diffX = 0, diffY = 0, diffZ = 0;
            for (unlong k = 1; k <= this->param.knn; k++) {
                diffX += this->cur_scan.at(i).at(j + k).x + this->cur_scan.at(i).at(j - k).x;
                diffY += this->cur_scan.at(i).at(j + k).y + this->cur_scan.at(i).at(j - k).y;
                diffZ += this->cur_scan.at(i).at(j + k).z + this->cur_scan.at(i).at(j - k).z;
            }
            diffX -= (this->param.knn * 2) * this->cur_scan.at(i).at(j).x;
            diffY -= (this->param.knn * 2) * this->cur_scan.at(i).at(j).y;
            diffZ -= (this->param.knn * 2) * this->cur_scan.at(i).at(j).z;
            this->cur_curve.at(i).at(j) = std::make_pair(true, diffX * diffX + diffY * diffY + diffZ * diffZ);
        }
    }
}

// This part filters out points that will not provide salient features. See
// paper for details
void LaserOdom::prefilter() {
    for (unlong i = 0; i < this->param.n_ring; i++) {
        if (this->cur_curve.at(i).size() != this->cur_scan.at(i).size()) {
            throw "Curvature and scan containers are different sizes!";
        }
        auto n_pts = this->cur_curve.at(i).size();
        int max_pts = (int) n_pts - (int) this->param.knn;
        if (n_pts < this->param.knn + 1) {
            this->filter.at(i).clear();
            continue;
        }
        for (unlong j = this->param.knn; j < max_pts; j++) {
            auto delforward = this->l2sqrd(this->cur_scan.at(i).at(j), this->cur_scan.at(i).at(j + 1));
            auto delback = this->l2sqrd(this->cur_scan.at(i).at(j), this->cur_scan.at(i).at(j - 1));
            // First section excludes any points who's score is likely caused
            // by occlusion
            if (delforward > this->param.occlusion_tol_2) {
                float d1 = std::sqrt(this->l2sqrd(this->cur_scan.at(i).at(j)));
                float d2 = std::sqrt(this->l2sqrd(this->cur_scan.at(i).at(j + 1)));
                auto unit1 = this->scale(this->cur_scan.at(i).at(j), 1.0 / d1);
                auto unit2 = this->scale(this->cur_scan.at(i).at(j + 1), 1.0 / d2);
                auto diff = std::sqrt(this->l2sqrd(unit1, unit2));
                if (diff < this->param.occlusion_tol) {
                    if (d1 > d2) {
                        for (unlong k = 0; k <= this->param.knn; k++) {
                            this->cur_curve.at(i).at(j - k).first = false;
                        }
                    } else {
                        for (unlong k = 1; k <= this->param.knn; k++) {
                            this->cur_curve.at(i).at(j + k).first = false;
                        }
                    }
                }
            }
            // This section excludes any points whose nearby surface is
            // near to parallel to the laser beam
            auto dis = this->l2sqrd(this->cur_scan.at(i).at(j));
            if ((delforward > (this->param.parallel_tol) * dis) && (delback > (this->param.parallel_tol * dis))) {
                this->cur_curve.at(i).at(j).first = false;
            }
        }
        // now store each selected point for sorting
        this->filter.at(i).clear();
        for (unlong j = this->param.knn; j < max_pts; j++) {
            if (this->cur_curve.at(i).at(j).first) {
                this->filter.at(i).emplace_back(j, this->cur_curve.at(i).at(j).second);
            }
        }
    }
}

}  // namespace wave
