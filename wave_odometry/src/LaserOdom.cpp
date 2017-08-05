#include <type_traits>
#include <algorithm>
#include <set>
#include <ceres/ceres.h>
#include <ceres/normal_prior.h>
#include <ceres/rotation.h>
#include <wave/matching/pointcloud_display.hpp>
#include "wave/odometry/loss_functions.hpp"
#include "wave/odometry/LaserOdom.hpp"

namespace wave {

const double ZERO_TOL = 1e-4;
// This amount of rotation or less is approximated as zero

double getDiff(double A[], double B[], int length) {
    double retval = 0;
    for (int i = 0; i < length; i++) {
        retval += (A[i] - B[i]) * (A[i] - B[i]);
    }
    return std::sqrt(retval);
}

void LaserOdom::flagNearbyPoints(const unlong ring, const unlong index) {
    for (unlong j = 0; j < this->param.knn; j++) {
        if (this->l2sqrd(this->cur_scan.at(ring).at(index + j),
                         this->cur_scan.at(ring).at(index + j + 1)) >
            this->param.keypt_radius) {
            break;
        }
        this->cur_curve.at(ring).at(index + j + 1).first = false;
    }
    for (unlong j = 0; j < this->param.knn; j++) {
        if (this->l2sqrd(this->cur_scan.at(ring).at(index - j),
                         this->cur_scan.at(ring).at(index - j - 1)) >
            this->param.keypt_radius) {
            break;
        }
        this->cur_curve.at(ring).at(index - j - 1).first = false;
    }
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
    this->prv_flats.resize(n_ring);
    this->prv_edges.resize(n_ring);
    this->flats.resize(n_ring);
    this->edges.resize(n_ring);
    this->edge_idx.resize(n_ring);
    this->flat_idx.resize(n_ring);
    for (size_t i = 0; i < n_ring; i++) {
        this->edge_idx.at(i) =
          new kd_tree_t(3,
                        this->prv_edges.at(i),
                        nanoflann::KDTreeSingleIndexAdaptorParams(10));
        this->flat_idx.at(i) =
          new kd_tree_t(3,
                        this->prv_flats.at(i),
                        nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }
    this->cur_translation = {0, 0, 0};
    this->cur_rotation = {0, 0, 0};
    if (params.visualize) {
        this->display = new PointCloudDisplay("laser odom", 0.05);
        this->display->startSpin();
        // Allocate space for visualization clouds
        this->prev_viz = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(
          new pcl::PointCloud<pcl::PointXYZI>);
        this->cur_viz = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(
          new pcl::PointCloud<pcl::PointXYZI>);
    }
    this->scale_lookup.resize(this->param.max_ticks + 1,
                              0);  // plus one is for 0 tick
    for (int i = 0; i <= this->param.max_ticks; i++) {
        this->scale_lookup.at(i) = (double) i / (double) this->param.max_ticks;
    }
}

LaserOdom::~LaserOdom() {
    if (this->param.visualize) {
        this->display->stopSpin();
        delete this->display;
    }
    for (size_t i = 0; i < this->param.n_ring; i++) {
        delete this->edge_idx.at(i);
        delete this->flat_idx.at(i);
    }
    this->cur_scan.clear();
    this->cur_curve.clear();
    this->filter.clear();
    this->prv_flats.clear();
    this->prv_edges.clear();
    this->edges.clear();
    this->flats.clear();
}

void LaserOdom::addPoints(const std::vector<PointXYZIR> &pts,
                          const int tick,
                          TimeType stamp) {
    if ((tick - this->prv_tick) < -2000) {  // tolerate minor nonlinearity error
        this->generateFeatures();  // generate features on the current scan
                                   // (empty is possible)
        if (this->initialized) {   // there is a set of previous features from
                                   // last scan
            double prev_translation[3], prev_rotation[3];
            for (int i = 0; i < this->param.opt_iters; i++) {
                if (i > 0) {
                    memcpy(prev_translation, this->cur_translation.data(), 24);
                    memcpy(prev_rotation, this->cur_rotation.data(), 24);
                }
                if (!this->match()) {
                    break;
                }
                LOG_INFO("\n%f \n%f \n%f \n%f \n%f \n%f",
                         this->cur_rotation.at(0),
                         this->cur_rotation.at(1),
                         this->cur_rotation.at(2),
                         this->cur_translation.at(0),
                         this->cur_translation.at(1),
                         this->cur_translation.at(2));
                if (i > 0) {
                    double diff =
                      getDiff(
                        prev_translation, this->cur_translation.data(), 3) +
                      getDiff(prev_rotation, this->cur_rotation.data(), 3);
                    if (diff < this->param.diff_tol) {
                        break;
                    }
                }
            }
            if (this->param.visualize) {
                this->updateViz();
            }
        }

        this->rollover(stamp);  // set previous features to current features and
                                // zero-out current scan
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

    for (uint16_t i = 0; i < this->param.n_ring; i++) {
        for (auto iter = this->prv_flats.at(i).points.begin();
             iter != this->prv_flats.at(i).points.end();
             iter++) {
            pcl::PointXYZI pt;
            pt.x = (*iter).at(0);
            pt.y = (*iter).at(1);
            pt.z = (*iter).at(2);
            pt.intensity = 1;
            this->prev_viz->push_back(pt);
        }
        for (auto iter = this->prv_edges.at(i).points.begin();
             iter != this->prv_edges.at(i).points.end();
             iter++) {
            pcl::PointXYZI pt;
            pt.x = (*iter).at(0);
            pt.y = (*iter).at(1);
            pt.z = (*iter).at(2);
            pt.intensity = 1;
            this->prev_viz->push_back(pt);
        }
    }

    // For the current features, need to transform them to start of scan
    std::vector<size_t> ret_indices(3);
    std::vector<double> out_dist_sqr(3);

    for (uint16_t i = 0; i < this->param.n_ring; i++) {
        for (auto iter = this->edges.at(i).begin();
             iter != this->edges.at(i).end();
             iter++) {
            double pt[3];
            double &scale = this->scale_lookup.at(iter->tick);
            double wvec[3] = {scale * this->cur_rotation[0],
                              scale * this->cur_rotation[1],
                              scale * this->cur_rotation[2]};
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
        for (auto iter = this->flats.at(i).begin();
             iter != this->flats.at(i).end();
             iter++) {
            double pt[3];
            double &scale = this->scale_lookup.at(iter->tick);
            double wvec[3] = {scale * this->cur_rotation[0],
                              scale * this->cur_rotation[1],
                              scale * this->cur_rotation[2]};
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
    std::this_thread::sleep_for(
      std::chrono::milliseconds((int) (10e3 * this->param.scan_period)));
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
        size_t cnt_edges = 0, cnt_flats = 0;
        for (uint16_t i = 0; i < this->param.n_ring; i++) {
            cnt_edges += this->prv_edges.at(i).points.size();
            cnt_flats += this->prv_flats.at(i).points.size();
        }
        if ((cnt_edges >= this->param.n_edge) &&
            (cnt_flats >= this->param.n_flat)) {
            this->initialized = true;
        }
    }
}

void LaserOdom::buildTrees() {
    for (uint16_t i = 0; i < this->param.n_ring; i++) {
        this->prv_edges.at(i).points.clear();
        this->prv_flats.at(i).points.clear();
    }
    Rotation rotation;
    Vec3 wvec(
      this->cur_rotation[0], this->cur_rotation[1], this->cur_rotation[2]);
    Vec3 trans(this->cur_translation[0],
               this->cur_translation[1],
               this->cur_translation[2]);

    for (uint16_t i = 0; i < this->param.n_ring; i++) {
        for (PointXYZIT pt : this->edges.at(i)) {
            double &scale_fwd = this->scale_lookup.at(pt.tick);

            double axis_angle[3] = {
              scale_fwd * wvec(0), scale_fwd * wvec(1), scale_fwd * wvec(2)};
            double rotated_pt[3] = {0};
            ceres::AngleAxisRotatePoint(axis_angle, pt.pt, rotated_pt);
            double trans_pt[3] = {rotated_pt[0] + scale_fwd * trans(0),
                                  rotated_pt[1] + scale_fwd * trans(1),
                                  rotated_pt[2] + scale_fwd * trans(2)};
            // now have point at the end of the scan, need to transform to
            // beginning
            axis_angle[0] = -wvec(0);
            axis_angle[1] = -wvec(1);
            axis_angle[2] = -wvec(2);
            ceres::AngleAxisRotatePoint(axis_angle, trans_pt, rotated_pt);
            double trans_total[3] = {trans(0), trans(1), trans(2)};
            double offset[3];
            ceres::AngleAxisRotatePoint(axis_angle, trans_total, offset);
            double final[3] = {rotated_pt[0] - offset[0],
                               rotated_pt[1] - offset[1],
                               rotated_pt[2] - offset[2]};

            this->prv_edges.at(i).points.push_back(
              std::array<double, 3>{final[0], final[1], final[2]});
        }

        for (PointXYZIT pt : this->flats.at(i)) {
            double &scale_fwd = this->scale_lookup.at(pt.tick);

            double axis_angle[3] = {
              scale_fwd * wvec(0), scale_fwd * wvec(1), scale_fwd * wvec(2)};
            double rotated_pt[3] = {0};
            ceres::AngleAxisRotatePoint(axis_angle, pt.pt, rotated_pt);
            double trans_pt[3] = {rotated_pt[0] + scale_fwd * trans(0),
                                  rotated_pt[1] + scale_fwd * trans(1),
                                  rotated_pt[2] + scale_fwd * trans(2)};
            // now have point at the beginning of the scan, need to transform to
            // end
            axis_angle[0] = -wvec(0);
            axis_angle[1] = -wvec(1);
            axis_angle[2] = -wvec(2);
            ceres::AngleAxisRotatePoint(axis_angle, trans_pt, rotated_pt);
            double trans_total[3] = {trans(0), trans(1), trans(2)};
            double offset[3];
            ceres::AngleAxisRotatePoint(axis_angle, trans_total, offset);
            double final[3] = {rotated_pt[0] - offset[0],
                               rotated_pt[1] - offset[1],
                               rotated_pt[2] - offset[2]};

            this->prv_flats.at(i).points.push_back(
              std::array<double, 3>{final[0], final[1], final[2]});
        }
    }
    for (uint16_t i = 0; i < this->param.n_ring; i++) {
        this->edge_idx.at(i)->buildIndex();
        this->flat_idx.at(i)->buildIndex();
    }

    this->cur_translation = {0, 0, 0};
    this->cur_rotation = {0, 0, 0};
}

bool LaserOdom::findPlanePoints(const Vec3 &query,
                                std::vector<int> *rings,
                                std::vector<int> *index) {
    // Need to query all rings and place report ring # and index # if points
    // make a good plane
    using ContType = std::pair<double, std::pair<size_t, uint16_t>>;
    std::vector<ContType> container;
    for (uint16_t ring_idx = 0; ring_idx < this->param.n_ring; ring_idx++) {
        std::vector<size_t> cur_ret_indices(2);
        std::vector<double> cur_out_dist_sqr(2);
        this->flat_idx.at(ring_idx)->knnSearch(
          query.data(), 2, &cur_ret_indices[0], &cur_out_dist_sqr[0]);
        for (size_t counter = 0; counter < cur_out_dist_sqr.size(); counter++) {
            if ((cur_out_dist_sqr.at(counter) <
                 this->param.max_correspondence_dist) &&
                (cur_out_dist_sqr.at(counter) > 0)) {
                container.emplace_back(cur_out_dist_sqr.at(counter), cur_ret_indices.at(counter), ring_idx);
            }
        }
    }
    // now sort container
    std::sort(container.begin(), container.end(),
    [](const ContType &left, const ContType &right) {
        return left.first < right.first;
    });
    // Now have sorted list of possible correpondences.
    // May want to revisit this later, but for now the policy will be
    // to pick the closest points if
    // - They are from neighbouring rings

}

bool LaserOdom::match() {
    const double min_init = 999999;
    ceres::Problem problem;
    ceres::LossFunction *p_LossFunction =
      new BisquareLoss(this->param.huber_delta);
    Rotation rotation;

    Vec3 wvec(
      this->cur_rotation[0], this->cur_rotation[1], this->cur_rotation[2]);
    Vec3 trans(this->cur_translation[0],
               this->cur_translation[1],
               this->cur_translation[2]);
    auto magnitude = wvec.norm();
    if (magnitude > ZERO_TOL) {
        wvec.normalize();
    }

    std::vector<size_t> ret_indices;
    std::vector<uint16_t> ret_rings;
    std::vector<double> out_dist_sqr;

    // residual blocks for edges
    for (uint16_t i = 0; i < this->param.n_ring; i++) {
        size_t idx;
        for (idx = 0; idx < this->edges.at(i).size(); idx++) {
            double &scale =
              this->scale_lookup.at(this->edges.at(i).at(idx).tick);

            Vec3 pt(this->edges.at(i).at(idx).pt[0],
                    this->edges.at(i).at(idx).pt[1],
                    this->edges.at(i).at(idx).pt[2]);
            if (magnitude < ZERO_TOL) {
                rotation.setIdentity();
            } else {
                rotation.setFromAngleAxis(scale * magnitude, wvec);
            }
            Vec3 query = rotation.rotate(pt) + scale * trans;

            ret_indices.resize(5);
            ret_rings.resize(5);
            out_dist_sqr.resize(5);
            int cur_idx = 0;
            this->edge_idx.at(i)->knnSearch(
              query.data(), 1, &ret_indices[cur_idx], &out_dist_sqr[cur_idx]);
            ret_rings[cur_idx] = i;
            cur_idx++;
            for (uint16_t j = 1; j < 3; j++) {
                if (((int) i - (int) j) >= 0) {
                    this->edge_idx.at(i - j)->knnSearch(query.data(),
                                                        1,
                                                        &ret_indices[cur_idx],
                                                        &out_dist_sqr[cur_idx]);
                    ret_rings[cur_idx] = i - j;
                    cur_idx++;
                }
                if (i + j < this->param.n_ring) {
                    this->edge_idx.at(i + j)->knnSearch(query.data(),
                                                        1,
                                                        &ret_indices[cur_idx],
                                                        &out_dist_sqr[cur_idx]);
                    ret_rings[cur_idx] = i + j;
                    cur_idx++;
                }
            }
            ret_indices.resize(cur_idx);
            out_dist_sqr.resize(cur_idx);
            ret_rings.resize(cur_idx);
            // now search through out_dist_sqr and pick the two closest points
            // The only restriction is that they must be from neighbouring rings
            double min1 = min_init, min2 = min_init;
            unsigned long index1, index2, ring1, ring2;
            for (uint16_t k = 0; k < out_dist_sqr.size(); k++) {
                if ((out_dist_sqr.at(k) >
                     this->param.max_correspondence_dist) ||
                    out_dist_sqr.at(k) == 0) {
                    continue;
                }
                if (out_dist_sqr.at(k) < min1) {
                    min2 = min1;
                    index2 = index1;
                    ring2 = ring1;
                    min1 = out_dist_sqr.at(k);
                    index1 = ret_indices.at(k);
                    ring1 = ret_rings.at(k);
                } else if (out_dist_sqr.at(k) < min2) {
                    min2 = out_dist_sqr.at(k);
                    index2 = ret_indices.at(k);
                    ring2 = ret_rings.at(k);
                }
            }

            if (min1 == min_init || min2 == min_init) {
                continue;
            }
            if (ring1 > ring2 ? (ring1 - ring2 != 1) : (ring2 - ring1 != 1)) {
                continue;
            }
            ceres::CostFunction *cost_function = new AnalyticalPointToLine(
              &(this->edges.at(i).at(idx).pt[0]),
              this->prv_edges.at(ring1).points.at(index1).data(),
              this->prv_edges.at(ring2).points.at(index2).data(),
              &(this->scale_lookup.at(this->edges.at(i).at(idx).tick)));
            problem.AddResidualBlock(cost_function,
                                     p_LossFunction,
                                     this->cur_rotation.data(),
                                     this->cur_translation.data());
        }

        // residuals blocks for flats
        for (size_t ind = 0; ind < this->flats.at(i).size(); ind++) {
            double &scale =
              this->scale_lookup.at(this->flats.at(i).at(ind).tick);

            Vec3 pt(this->flats.at(i).at(ind).pt[0],
                    this->flats.at(i).at(ind).pt[1],
                    this->flats.at(i).at(ind).pt[2]);
            if (magnitude < ZERO_TOL) {
                rotation.setIdentity();
            } else {
                rotation.setFromAngleAxis(scale * magnitude, wvec);
            }
            Vec3 query = rotation.rotate(pt) + scale * trans;

            ret_indices.resize(10);
            ret_rings.resize(10);
            out_dist_sqr.resize(10);
            int cur_idx = 0;
            this->flat_idx.at(i)->knnSearch(
              query.data(), 2, &ret_indices[cur_idx], &out_dist_sqr[cur_idx]);
            ret_rings[cur_idx] = i;
            cur_idx += 2;
            for (uint16_t j = 1; j < 3; j++) {
                if (((int) i - (int) j) >=
                    0) {  // they are unsigned so wrap instead of <0
                    this->flat_idx.at(i - j)->knnSearch(query.data(),
                                                        2,
                                                        &ret_indices[cur_idx],
                                                        &out_dist_sqr[cur_idx]);
                    ret_rings[cur_idx] = i - j;
                    ret_rings[cur_idx + 1] = i - j;
                    cur_idx += 2;
                }
                if (i + j < this->param.n_ring) {
                    this->flat_idx.at(i + j)->knnSearch(query.data(),
                                                        2,
                                                        &ret_indices[cur_idx],
                                                        &out_dist_sqr[cur_idx]);
                    ret_rings[cur_idx] = i + j;
                    ret_rings[cur_idx + 1] = i + j;
                    cur_idx += 2;
                }
            }
            ret_indices.resize(cur_idx);
            out_dist_sqr.resize(cur_idx);
            ret_rings.resize(cur_idx);
            // now search through out_dist_sqr and pick the three closest points
            // The only restriction is that they must be from neighbouring rings
            std::vector<double> min(3, min_init);
            std::vector<unsigned long> index(3), ring(3);
            for (uint16_t k = 0; k < out_dist_sqr.size(); k++) {
                if ((out_dist_sqr.at(k) >
                     this->param.max_correspondence_dist) ||
                    out_dist_sqr.at(k) == 0) {
                    continue;
                }
                for (int l = 0; l < 3; l++) {
                    if (out_dist_sqr.at(k) < min[l]) {
                        for (int n = 2; n > l; n--) {
                            min[n] = min[n - 1];
                            ring[n] = ring[n - 1];
                            index[n] = index[n - 1];
                        }
                        min[l] = out_dist_sqr.at(k);
                        index[l] = ret_indices.at(k);
                        ring[l] = ret_rings.at(k);
                        break;
                    }
                }
            }

            std::set<double> dists(min.cbegin(), min.cend());
            if (dists.count(min_init) > 0) {
                continue;
            }

            // Want each set of points to be from neighbouring scan lines
            std::set<unsigned long> ring_set(ring.cbegin(), ring.cend());
            if (ring_set.size() != 2) {
                continue;
            }
            ceres::CostFunction *cost_function = new AnalyticalPointToPlane(
              &(this->flats.at(i).at(ind).pt[0]),
              this->prv_flats.at(ring.at(0)).points.at(index.at(0)).data(),
              this->prv_flats.at(ring.at(1)).points.at(index.at(1)).data(),
              this->prv_flats.at(ring.at(2)).points.at(index.at(2)).data(),
              &(this->scale_lookup.at(this->flats.at(i).at(ind).tick)));
            problem.AddResidualBlock(cost_function,
                                     p_LossFunction,
                                     this->cur_rotation.data(),
                                     this->cur_translation.data());
        }
    }

    // Add a normal prior to provide some regularization
    // This is in effect a constant velocity model
    //    ceres::Matrix stiffness(6, 6);
    //    stiffness.setIdentity();
    //    stiffness(0, 0) = 0.0000001;
    //    stiffness(1, 1) = 0.0000001;
    //    stiffness(2, 2) = 0.0000001;
    //    stiffness(3, 3) = 0.0000001;
    //    stiffness(4, 4) = 0.0000001;
    //    stiffness(5, 5) = 0.0000001;
    //
    //    double vec[6];
    //    memcpy(vec, this->cur_transform.data(), 6 * 8);
    //    ceres::VectorRef initial(vec, 6, 1);
    //    ceres::NormalPrior *diff_cost_function =
    //      new ceres::NormalPrior(stiffness, initial);
    //    problem.AddResidualBlock(
    //      diff_cost_function, NULL, this->cur_transform.data());


    // problem.SetParameterBlockConstant(this->cur_rotation.data());

    ceres::Solver::Options options;
    std::vector<int> iterations = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 300;
    options.function_tolerance = 1e-10;
    options.parameter_tolerance = 1e-10;
    options.num_threads = 1;
    options.num_linear_solver_threads = 1;
    // options.trust_region_minimizer_iterations_to_dump = iterations;
    options.trust_region_problem_dump_format_type =
      ceres::DumpFormatType::TEXTFILE;
    ceres::Solver::Summary summary;
    auto max_residuals =
      this->param.n_ring * (this->param.n_flat + this->param.n_edge);
    if (problem.NumResidualBlocks() < max_residuals * 0.2) {
        LOG_ERROR("Less than expected residuals, resetting");
        LOG_ERROR("%d residuals, threshold is %f",
                  problem.NumResidualBlocks(),
                  max_residuals * 0.2);
        this->cur_translation = {0, 0, 0};
        this->cur_rotation = {0, 0, 0};
        this->initialized = false;
        return false;
    } else {
        ceres::Solve(options, &problem, &summary);
        LOG_INFO("%s", summary.FullReport().c_str());
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
                  [](const std::pair<unlong, float> lhs,
                     const std::pair<unlong, float> rhs) {
                      return lhs.second < rhs.second;
                  });
        int edge_cnt = 0, flat_cnt = 0;

        // Pick out edge points
        for (auto iter = this->filter.at(i).rbegin();
             iter < this->filter.at(i).rend();
             iter++) {
            if ((iter->second < this->param.edge_tol) ||
                (edge_cnt >= this->param.n_edge)) {
                break;
            } else if (this->cur_curve.at(i).at(iter->first).first) {
                this->edges.at(i).emplace_back(
                  this->cur_scan.at(i).at(iter->first).x,
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
        for (auto iter = this->filter.at(i).begin();
             iter < this->filter.at(i).end();
             iter++) {
            if ((iter->second > this->param.flat_tol) ||
                (flat_cnt >= this->param.n_flat)) {
                break;
            } else if (this->cur_curve.at(i).at(iter->first).first) {
                this->flats.at(i).emplace_back(
                  this->cur_scan.at(i).at(iter->first).x,
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
    this->new_features = true;
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
                diffX += this->cur_scan.at(i).at(j + k).x +
                         this->cur_scan.at(i).at(j - k).x;
                diffY += this->cur_scan.at(i).at(j + k).y +
                         this->cur_scan.at(i).at(j - k).y;
                diffZ += this->cur_scan.at(i).at(j + k).z +
                         this->cur_scan.at(i).at(j - k).z;
            }
            diffX -= (this->param.knn * 2) * this->cur_scan.at(i).at(j).x;
            diffY -= (this->param.knn * 2) * this->cur_scan.at(i).at(j).y;
            diffZ -= (this->param.knn * 2) * this->cur_scan.at(i).at(j).z;
            this->cur_curve.at(i).at(j) = std::make_pair(
              true, diffX * diffX + diffY * diffY + diffZ * diffZ);
        }
    }
}

// This part filters out points that will not provide salient features. See
// paper for details
void LaserOdom::prefilter() {
    for (unlong i = 0; i < this->param.n_ring; i++) {
        auto n_pts = this->cur_curve.at(i).size();
        int max_pts = (int) n_pts - (int) this->param.knn;
        if (n_pts < this->param.knn + 1) {
            continue;
        }
        for (unlong j = this->param.knn; j < max_pts; j++) {
            auto delforward = this->l2sqrd(this->cur_scan.at(i).at(j),
                                           this->cur_scan.at(i).at(j + 1));
            auto delback = this->l2sqrd(this->cur_scan.at(i).at(j),
                                        this->cur_scan.at(i).at(j - 1));
            // First section excludes any points who's score is likely caused
            // by occlusion
            if (delforward > this->param.occlusion_tol) {
                float d1 = std::sqrt(this->l2sqrd(this->cur_scan.at(i).at(j)));
                float d2 =
                  std::sqrt(this->l2sqrd(this->cur_scan.at(i).at(j + 1)));
                auto unit1 = this->scale(this->cur_scan.at(i).at(j), 1 / d1);
                auto unit2 =
                  this->scale(this->cur_scan.at(i).at(j + 1), 1 / d2);
                auto diff = std::sqrt(this->l2sqrd(unit1, unit2));
                if (diff < this->param.occlusion_tol) {
                    if (d1 > d2) {
                        // todo(ben) See if this <= is actually correct
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
            // near to parallel to the laser
            auto dis = this->l2sqrd(this->cur_scan.at(i).at(j));
            if ((delforward > (this->param.parallel_tol) * dis) &&
                (delback > (this->param.parallel_tol * dis))) {
                this->cur_curve.at(i).at(j).first = false;
            }
        }
        // now store each selected point for sorting
        this->filter.at(i).resize(max_pts);
        unlong cnt = 0;
        for (unlong j = 0; j < n_pts; j++) {
            if (this->cur_curve.at(i).at(j).first) {
                this->filter.at(i).at(cnt).first = j;
                this->filter.at(i).at(cnt).second =
                  this->cur_curve.at(i).at(j).second;
                cnt++;
            }
        }
        this->filter.at(i).resize(cnt);
    }
}

}  // namespace wave
