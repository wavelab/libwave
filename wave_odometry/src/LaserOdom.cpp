#include <type_traits>
#include <ceres/ceres.h>
#include <ceres/internal/eigen.h>
#include <ceres/normal_prior.h>
#include <ceres/rotation.h>
#include <wave/matching/pointcloud_display.hpp>
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
    this->edge_idx = new kd_tree_t(
      3, this->prv_edges, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    this->flat_idx = new kd_tree_t(
      3, this->prv_flats, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    this->cur_transform = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4};
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

void LaserOdom::addPoints(const std::vector<PointXYZIR> &pts,
                          const int tick,
                          TimeType stamp) {
    if ((tick - this->prv_tick) < -2000) {  // tolerate minor nonlinearity error
        this->generateFeatures();  // generate features on the current scan
                                   // (empty is possible)
        if (this->initialized) {   // there is a set of previous features from
                                   // last scan
            double prev_transform[6];
            for (int i = 0; i < this->param.opt_iters; i++) {
                if (i > 0) {
                    memcpy(prev_transform, this->cur_transform.data(), 48);
                }
                if (!this->match()) {
                    break;
                }
                LOG_INFO("\n%f \n%f \n%f \n%f \n%f \n%f",
                         this->cur_transform.at(0),
                         this->cur_transform.at(1),
                         this->cur_transform.at(2),
                         this->cur_transform.at(3),
                         this->cur_transform.at(4),
                         this->cur_transform.at(5));
                if (i > 0) {
                    double diff =
                      getDiff(prev_transform, this->cur_transform.data(), 6);
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

    for (auto iter = this->prv_flats.points.begin();
         iter != this->prv_flats.points.end();
         iter++) {
        pcl::PointXYZI pt;
        pt.x = (*iter).at(0);
        pt.y = (*iter).at(1);
        pt.z = (*iter).at(2);
        pt.intensity = 1;
        this->prev_viz->push_back(pt);
    }
    for (auto iter = this->prv_edges.points.begin();
         iter != this->prv_edges.points.end();
         iter++) {
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
//    int arb_id_counter = 0;

    for (auto iter = this->edges.begin(); iter != this->edges.end(); iter++) {
        double pt[3];
        double &scale = this->scale_lookup.at(iter->tick);
        double wvec[3] = {scale * this->cur_transform[0],
                          scale * this->cur_transform[1],
                          scale * this->cur_transform[2]};
        ceres::AngleAxisRotatePoint(wvec, iter->pt, pt);
        pt[0] += scale * this->cur_transform[3];
        pt[1] += scale * this->cur_transform[4];
        pt[2] += scale * this->cur_transform[5];

        //        this->edge_idx->knnSearch(pt, 2, &ret_indices[0],
        //        &out_dist_sqr[0]);
        //
        //        if (out_dist_sqr.at(1) < this->param.max_correspondence_dist)
        //        {
        //            // draw correspondences
        //            this->display->addLine(
        //              pcl::PointXYZ(pt[0], pt[1], pt[2]),
        //              pcl::PointXYZ(this->prv_edges.points.at(ret_indices[0]).at(0),
        //                            this->prv_edges.points.at(ret_indices[0]).at(1),
        //                            this->prv_edges.points.at(ret_indices[0]).at(2)),
        //              arb_id_counter,
        //              ret_indices[0]);
        //            this->display->addLine(
        //              pcl::PointXYZ(pt[0], pt[1], pt[2]),
        //              pcl::PointXYZ(this->prv_edges.points.at(ret_indices[1]).at(0),
        //                            this->prv_edges.points.at(ret_indices[1]).at(1),
        //                            this->prv_edges.points.at(ret_indices[1]).at(2)),
        //              arb_id_counter,
        //              ret_indices[1]);
        //            arb_id_counter++;
        //        }

        pcl::PointXYZI point;
        point.x = pt[0];
        point.y = pt[1];
        point.z = pt[2];
        point.intensity = 2;
        this->prev_viz->push_back(point);
    }
    for (auto iter = this->flats.begin(); iter != this->flats.end(); iter++) {
        double pt[3];
        double &scale = this->scale_lookup.at(iter->tick);
        double wvec[3] = {scale * this->cur_transform[0],
                          scale * this->cur_transform[1],
                          scale * this->cur_transform[2]};
        ceres::AngleAxisRotatePoint(wvec, iter->pt, pt);
        pt[0] += scale * this->cur_transform[3];
        pt[1] += scale * this->cur_transform[4];
        pt[2] += scale * this->cur_transform[5];

        //        this->flat_idx->knnSearch(pt, 3, &ret_indices[0],
        //        &out_dist_sqr[0]);
        //
        //        if (out_dist_sqr.at(2) < this->param.max_correspondence_dist)
        //        {
        //            // draw correspondences
        //            this->display->addLine(
        //              pcl::PointXYZ(pt[0], pt[1], pt[2]),
        //              pcl::PointXYZ(this->prv_flats.points.at(ret_indices[0]).at(0),
        //                            this->prv_flats.points.at(ret_indices[0]).at(1),
        //                            this->prv_flats.points.at(ret_indices[0]).at(2)),
        //              arb_id_counter,
        //              ret_indices[0] + 30000);
        //            this->display->addLine(
        //              pcl::PointXYZ(pt[0], pt[1], pt[2]),
        //              pcl::PointXYZ(this->prv_flats.points.at(ret_indices[1]).at(0),
        //                            this->prv_flats.points.at(ret_indices[1]).at(1),
        //                            this->prv_flats.points.at(ret_indices[1]).at(2)),
        //              arb_id_counter,
        //              ret_indices[1] + 30000);
        //            this->display->addLine(
        //              pcl::PointXYZ(pt[0], pt[1], pt[2]),
        //              pcl::PointXYZ(this->prv_flats.points.at(ret_indices[2]).at(0),
        //                            this->prv_flats.points.at(ret_indices[2]).at(1),
        //                            this->prv_flats.points.at(ret_indices[2]).at(2)),
        //              arb_id_counter,
        //              ret_indices[2] + 30000);
        //            arb_id_counter++;
        //        }

        pcl::PointXYZI point;
        point.x = pt[0];
        point.y = pt[1];
        point.z = pt[2];
        point.intensity = 2;
        this->prev_viz->push_back(point);
    }
    this->display->addPointcloud(this->prev_viz, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
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
        if ((this->prv_edges.points.size() >= this->param.n_edge) &&
            (this->prv_flats.points.size() >= this->param.n_flat)) {
            this->initialized = true;
        }
    }
}

void LaserOdom::buildTrees() {
    this->prv_edges.points.clear();
    this->prv_flats.points.clear();

    Rotation rotation;
    Vec3 wvec(-this->cur_transform[0],
              -this->cur_transform[1],
              -this->cur_transform[2]);
    Vec3 trans(
      this->cur_transform[3], this->cur_transform[4], this->cur_transform[5]);
    auto magnitude = wvec.norm();
    wvec.normalize();

    for (PointXYZIT pt : this->edges) {
        double &scale = this->scale_lookup.at(this->param.max_ticks - pt.tick);
        if (scale * magnitude > ZERO_TOL) {
            rotation.setFromAngleAxis(scale * magnitude, wvec);
        } else {
            rotation.setIdentity();
        }
        Vec3 temp(pt.pt[0], pt.pt[1], pt.pt[2]);
        Vec3 transfed = rotation.rotate(temp - scale * trans);
        this->prv_edges.points.push_back(
          std::array<double, 3>{transfed(0), transfed(1), transfed(2)});
    }

    for (PointXYZIT pt : this->flats) {
        double &scale = this->scale_lookup.at(this->param.max_ticks - pt.tick);
        if (scale * magnitude > ZERO_TOL) {
            rotation.setFromAngleAxis(scale * magnitude, wvec);
        } else {
            rotation.setIdentity();
        }

        Vec3 temp(pt.pt[0], pt.pt[1], pt.pt[2]);
        Vec3 transfed = rotation.rotate(temp - scale * trans);
        this->prv_flats.points.push_back(
          std::array<double, 3>{transfed(0), transfed(1), transfed(2)});
    }
    this->edge_idx->buildIndex();
    this->flat_idx->buildIndex();
}

bool LaserOdom::match() {
    ceres::Problem problem;
    ceres::LossFunction *p_LossFunction =
      new ceres::HuberLoss(this->param.huber_delta);
    Rotation rotation;

    Vec3 wvec(
      this->cur_transform[0], this->cur_transform[1], this->cur_transform[2]);
    Vec3 trans(
      this->cur_transform[3], this->cur_transform[4], this->cur_transform[5]);
    auto magnitude = wvec.norm();
    if (magnitude > ZERO_TOL) {
        // normalize rotation
        wvec = wvec / magnitude;
    }

    // Only need up to 3 nearest neighbours for point to plane matching
    // Point to line requires only 2
    std::vector<size_t> ret_indices(3);
    std::vector<double> out_dist_sqr(3);

    // residual blocks for edges
    size_t idx;
    for (idx = 0; idx < this->edges.size(); idx++) {
        double &scale = this->scale_lookup.at(this->edges.at(idx).tick);

        Vec3 pt(this->edges.at(idx).pt[0],
                this->edges.at(idx).pt[1],
                this->edges.at(idx).pt[2]);
        if (magnitude < ZERO_TOL) {
            rotation.setIdentity();
        } else {
            rotation.setFromAngleAxis(scale * magnitude, wvec);
        }
        Vec3 query = rotation.rotate(pt) + scale * trans;

        this->edge_idx->knnSearch(
          query.data(), 2, &ret_indices[0], &out_dist_sqr[0]);

        if (out_dist_sqr.at(1) > this->param.max_correspondence_dist) {
            continue;
        }
        ceres::CostFunction *cost_function = PointToLineError::Create();
        problem.AddResidualBlock(
          cost_function,
          NULL,  // p_LossFunction,
          this->cur_transform.data(),
          &(this->edges.at(idx).pt[0]),
          this->prv_edges.points.at(ret_indices.at(0)).data(),
          this->prv_edges.points.at(ret_indices.at(1)).data(),
          &scale);
        problem.SetParameterBlockConstant(&(this->edges.at(idx).pt[0]));
        problem.SetParameterBlockConstant(
          this->prv_edges.points.at(ret_indices.at(0)).data());
        problem.SetParameterBlockConstant(
          this->prv_edges.points.at(ret_indices.at(1)).data());
        problem.SetParameterBlockConstant(&scale);
    }

    // residuals blocks for flats
    for (size_t ind = 0; ind < this->flats.size(); ind++) {
        double &scale = this->scale_lookup.at(this->flats.at(ind).tick);

        Vec3 pt(this->flats.at(ind).pt[0],
                this->flats.at(ind).pt[1],
                this->flats.at(ind).pt[2]);
        if (magnitude < ZERO_TOL) {
            rotation.setIdentity();
        } else {
            rotation.setFromAngleAxis(scale * magnitude, wvec);
        }
        Vec3 query = rotation.rotate(pt) + scale * trans;

        this->flat_idx->knnSearch(
          query.data(), 3, &ret_indices[0], &out_dist_sqr[0]);

        if (out_dist_sqr.at(2) > this->param.max_correspondence_dist) {
            continue;
        }

        ceres::CostFunction *cost_function = PointToPlaneError::Create();
        problem.AddResidualBlock(
          cost_function,
          NULL,  // p_LossFunction,
          this->cur_transform.data(),
          &(this->flats.at(ind).pt[0]),
          this->prv_flats.points.at(ret_indices.at(0)).data(),
          this->prv_flats.points.at(ret_indices.at(1)).data(),
          this->prv_flats.points.at(ret_indices.at(2)).data(),
          &scale);
        problem.SetParameterBlockConstant(&(this->flats.at(ind).pt[0]));
        problem.SetParameterBlockConstant(
          this->prv_flats.points.at(ret_indices.at(0)).data());
        problem.SetParameterBlockConstant(
          this->prv_flats.points.at(ret_indices.at(1)).data());
        problem.SetParameterBlockConstant(
          this->prv_flats.points.at(ret_indices.at(2)).data());
        problem.SetParameterBlockConstant(&scale);
    }

    // Add a normal prior to provide some regularization
    // This is in effect a constant velocity model
    //    ceres::Matrix stiffness(6, 6);
    //    stiffness.setIdentity();
    //    double vec[6];
    //    memcpy(vec, this->cur_transform.data(), 6 * 8);
    //    ceres::VectorRef initial(vec, 6, 1);
    //    ceres::NormalPrior *diff_cost_function = new
    //    ceres::NormalPrior(stiffness, initial);
    //    problem.AddResidualBlock(diff_cost_function, NULL,
    //    this->cur_transform.data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    auto max_residuals =
      this->param.n_ring * (this->param.n_flat + this->param.n_edge);
    if (problem.NumResidualBlocks() < max_residuals * 0.2) {
        LOG_ERROR("Less than expected residuals, resetting");
        this->cur_transform = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4};
        this->initialized = false;
        return false;
    } else {
        ceres::Solve(options, &problem, &summary);
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
    this->edges.clear();
    this->flats.clear();
    for (unlong i = 0; i < this->param.n_ring; i++) {
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
                this->edges.emplace_back(
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
                this->flats.emplace_back(
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
