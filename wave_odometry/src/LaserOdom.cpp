#include <type_traits>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "wave/odometry/LaserOdom.hpp"

namespace wave {

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
    this->cur_transform = {};
}

void LaserOdom::addPoints(const std::vector<PointXYZIR> &pts,
                          const int tick,
                          TimeType stamp) {
    if (tick == 0) {  // Current scan has finished
        this->generateFeatures();
        if (this->initialized) {
            for (int i = 0; i < this->param.opt_iters; i++) {
                // Should add a way to get out early if the correspondences
                // don't change
                this->match();
            }
        } else {
            this->initialized = true;
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
}

void LaserOdom::buildTrees() {
    this->prv_edges.points.clear();
    this->prv_flats.points.clear();

    Rotation rotation;
    Vec3 wvec(
      this->cur_transform[0], this->cur_transform[1], this->cur_transform[2]);
    Vec3 trans(
      this->cur_transform[3], this->cur_transform[4], this->cur_transform[5]);
    auto magnitude = wvec.norm();
    wvec.normalize();

    for (PointXYZIT pt : this->edges) {
        auto scale = ((float) pt.tick / (float) this->param.max_ticks) *
                     this->param.scan_period;
        if (scale * magnitude > 1e-5) {
            rotation.setFromAngleAxis(scale * magnitude, wvec);
        } else {
            rotation.setIdentity();
        }
        Vec3 temp(pt.pt[0], pt.pt[1], pt.pt[2]);
        Vec3 transfed = rotation.rotate(temp) + scale * trans;
        this->prv_edges.points.push_back(
          std::array<double, 3>{transfed(0), transfed(1), transfed(2)});
    }

    for (PointXYZIT pt : this->flats) {
        auto scale = ((float) pt.tick / (float) this->param.max_ticks) *
                     this->param.scan_period;
        if (scale * magnitude > 1e-5) {
            rotation.setFromAngleAxis(scale * magnitude, wvec);
        } else {
            rotation.setIdentity();
        }

        Vec3 temp(pt.pt[0], pt.pt[1], pt.pt[2]);
        Vec3 transfed = rotation.rotate(temp) + scale * trans;
        this->prv_flats.points.push_back(
          std::array<double, 3>{transfed(0), transfed(1), transfed(2)});
    }
    this->edge_idx->buildIndex();
    this->flat_idx->buildIndex();
}

void LaserOdom::match() {
    ceres::Problem problem;
    ceres::LossFunction *p_LossFunction =
      new ceres::HuberLoss(this->param.huber_delta);
    Rotation rotation;

    Vec3 wvec(-this->cur_transform[0],
              -this->cur_transform[1],
              -this->cur_transform[2]);
    Vec3 trans(
      this->cur_transform[3], this->cur_transform[4], this->cur_transform[5]);
    auto magnitude = wvec.norm();

    // Only need up to 3 nearest neighbours for point to plane matching
    // Point to line requires only 2
    std::vector<size_t> ret_indices(3);
    std::vector<double> out_dist_sqr(3);
    // residual blocks for edges
    for (size_t idx = 0; idx < this->edges.size(); idx++) {
        double scale =
          ((float) this->edges.at(idx).tick / (float) this->param.max_ticks) *
          this->param.scan_period;

        Vec3 pt(this->edges.at(idx).pt[0],
                this->edges.at(idx).pt[1],
                this->edges.at(idx).pt[2]);
        if (magnitude < 1e-4) {
            rotation.setIdentity();
        } else {
            rotation.setFromAngleAxis(scale * magnitude, wvec);
        }
        Vec3 query = rotation.rotate(pt - scale * trans);

        this->edge_idx->knnSearch(
          query.data(), 2, &ret_indices[0], &out_dist_sqr[0]);

        if (out_dist_sqr.at(1) > this->param.max_correspondence_dist) {
            continue;
        }

        ceres::CostFunction *cost_function = PointToLineError::Create();
        problem.AddResidualBlock(
          cost_function,
          p_LossFunction,
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
    for (size_t idx = 0; idx < this->flats.size(); idx++) {
        double scale =
          ((float) this->flats.at(idx).tick / (float) this->param.max_ticks) *
          this->param.scan_period;

        Vec3 pt(this->flats.at(idx).pt[0],
                this->flats.at(idx).pt[1],
                this->flats.at(idx).pt[2]);
        if (magnitude < 1e-4) {
            rotation.setIdentity();
        } else {
            rotation.setFromAngleAxis(scale * magnitude, wvec);
        }
        Vec3 query = rotation.rotate(pt - scale * trans);

        this->edge_idx->knnSearch(
          query.data(), 3, &ret_indices[0], &out_dist_sqr[0]);

        if (out_dist_sqr.at(2) > this->param.max_correspondence_dist) {
            continue;
        }

        ceres::CostFunction *cost_function = PointToPlaneError::Create();
        problem.AddResidualBlock(
          cost_function,
          p_LossFunction,
          this->cur_transform.data(),
          &(this->flats.at(idx).pt[0]),
          this->prv_flats.points.at(ret_indices.at(0)).data(),
          this->prv_flats.points.at(ret_indices.at(1)).data(),
          this->prv_flats.points.at(ret_indices.at(2)).data(),
          &scale);
        problem.SetParameterBlockConstant(&(this->flats.at(idx).pt[0]));
        problem.SetParameterBlockConstant(
          this->prv_flats.points.at(ret_indices.at(0)).data());
        problem.SetParameterBlockConstant(
          this->prv_flats.points.at(ret_indices.at(1)).data());
        problem.SetParameterBlockConstant(
          this->prv_flats.points.at(ret_indices.at(2)).data());
        problem.SetParameterBlockConstant(&scale);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
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
        for (auto iter = this->filter.at(i).end();
             iter > this->filter.at(i).begin();
             iter--) {
            if ((iter->second < this->param.edge_tol) ||
                (edge_cnt >= this->param.n_edge)) {
                if (iter == this->filter.at(i).end()) {
                    continue;
                }
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
        int max_pts = (int)n_pts - (int)this->param.knn;
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
