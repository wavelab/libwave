#include "slam/optimization/optimizers/ransac.hpp"


namespace slam {

RANSAC::RANSAC(void)
{
    this->configured = false;

    this->max_iter = 0;
    this->thresh_ratio = 1.0;
    this->thresh_dist = 0.0;

    this->iter = 0;
    this->max_inliers = 0;
    this->model_params[0] = 0.0;
    this->model_params[1] = 0.0;
}

int RANSAC::configure(int max_iter, double thresh_ratio, double thresh_dist)
{
    this->configured = true;

    this->max_iter = max_iter;
    this->thresh_ratio = thresh_ratio;
    this->thresh_dist = thresh_dist;

    this->iter = 0;
    this->max_inliers = 0;
    this->model_params[0] = 0.0;
    this->model_params[1] = 0.0;

    return 0;
}

int RANSAC::randomSample(MatX &data, Vec2 &sample)
{
    int rand_index;

    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (data.rows() != 2) {
        return -2;
    } else if (data.cols() < 2) {
        return -3;
    }

    // random sample
    rand_index = rand() % (data.cols() - 1);
    sample << data(0, rand_index), data(1, rand_index);

    return 0;
}

int RANSAC::computeDistances(MatX &data, Vec2 &p1, Vec2 &p2, VecX &dists)
{
    int indicies[2];
    double dx, dy, dist, pdiff_norm;
    Vec2 pdiff, pdiff_unit, norm;
    MatX repmat(2, 1);

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // diff between p2 and p1
    pdiff = p2 - p1;
    pdiff_norm = pdiff.norm();
    pdiff_unit << (pdiff(0) / pdiff_norm), (pdiff(1) / pdiff_norm);

    // Ax + By + C = 0; A = -pdiff_unit(1), B = pdiff_unit(0)
    norm << -pdiff_unit(1), pdiff_unit(0);

    // calc dist between line formed by two random and all other points
    repmat << p1(0), p1(1);
    dists = norm.transpose() * (data - repmat.replicate(1, data.cols()));

    return 0;
}

int RANSAC::computeInliers(VecX &dists)
{
    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // compute inliers
    this->inliers.clear();
    for (int i = 0; i < dists.rows(); i++) {
        if (fabs(dists[i]) <= this->thresh_dist) {
            this->inliers.push_back(i);
        }
    }

    return 0;
}

int RANSAC::update(Vec2 &p1, Vec2 &p2)
{
    int conditions_met;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // setup
    conditions_met = 0;
    conditions_met += (this->inliers.size() >= this->threshold) ? 1 : 0;
    conditions_met += (this->inliers.size() > this->max_inliers) ? 1 : 0;

    if (conditions_met == 2) {
        this->max_inliers = this->inliers.size();
        this->model_params[0] = (p2(1) - p1(1)) / (p2(0) - p1(0));
        this->model_params[1] = p1(1) - this->model_params[0] * p1(0);
    }

    return 0;
}

int RANSAC::printStats(void)
{
    printf("iter: %d\t", this->iter);
    printf("inliers: %d\t", this->max_inliers);
    printf("m: %f\t", this->model_params[0]);
    printf("c: %f\n", this->model_params[1]);
}

int RANSAC::optimize(MatX &data)
{
    Vec2 p1;
    Vec2 p2;
    VecX dists;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // setup
    this->threshold = round(this->thresh_ratio * data.cols());

    // optimize
    for (this->iter = 0; this->iter < this->max_iter; this->iter++) {
        // random sample 2 points
        this->randomSample(data, p1);
        this->randomSample(data, p2);

        // compute distances and inliers
        this->computeDistances(data, p1, p2, dists);
        this->computeInliers(dists);

        // update better model if found
        this->update(p1, p2);
        // this->printStats();
    }

    return 0;
}

}  // end of slam namespace
