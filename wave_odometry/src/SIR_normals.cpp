#include <wave/odometry/geometry/SRI_normals.hpp>

namespace wave {

NormalEstimator::NormalEstimator(wave::NormalEstimatorParams param) : param(param) {
    long cols = std::lround(std::ceil(param.azimuth_range / param.azimuth_resolution));
    long rows = std::lround(std::ceil((param.max_elevation - param.min_elevation)
            / param.elevation_resolution));

    this->normal_mask = Eigen::Tensor<bool, 2>(rows, cols);
    this->normal_mask.setConstant(false);
    this->interpolation_mask = Eigen::Tensor<bool, 2>(rows, cols);
    this->interpolation_mask.setConstant(false);
    this->interpolated_image = Eigen::Tensor<float, 2>(rows, cols);
    this->interpolated_image.setConstant(0);
}

void NormalEstimator::setInput(const Eigen::Tensor<float, 2> &input_scan, long n_rings) {
    this->starting_azimuth = 0;
    for (int i = 0; i < n_rings; ++i) {
        if (i >= input_scan.dimension(1)) {
            throw std::runtime_error("Empty (or nearly empty) input scan inside Normal Estimator");
        }
        if (this->starting_azimuth < input_scan(0,i)) {
            this->starting_azimuth = input_scan(0,i);
        }
    }
    long pt_ctr = 0;
    long ctr = 0;
    bool past_zero = false;
    for (long pt_idx = 0; pt_idx < input_scan.dimension(1); ++pt_idx) {
        float cur_az = input_scan(0, pt_idx);
        if (cur_az > M_PI || past_zero) {
            cur_az = (float) (cur_az - 2 * M_PI);
            if (!past_zero) {
                if(++ctr > n_rings) {
                    past_zero = true;
                }
            }
        } else {
            ctr = 0;
        }
        if (std::isnan(input_scan(2, pt_idx))) {
            continue;
        }
        float r_nf = (input_scan(1, pt_idx) - this->param.min_elevation)
            / this->param.elevation_resolution;
        float c_nf = (this->starting_azimuth - cur_az)
                     / this->param.azimuth_resolution;

        // consisten with rows for elevation and cols for azimuth
        Triangulation::Point new_pt(input_scan(1,pt_idx), cur_az);
        this->points.emplace_back(std::make_pair(new_pt, input_scan(2,pt_idx)));
        ++pt_ctr;

        long r_n = std::lround(std::floor(r_nf));
        long c_n = std::lround(std::floor(c_nf));
        this->normal_mask.slice(ar2{r_n, c_n}, ar2{2, 2}).setConstant(true);
        long r_i = r_n - 3 < 0 ? 0 : r_n - 3;
        long c_i = c_n - 3 < 0 ? 0 : c_n - 3;
        long r_extents = r_n + 4 >= normal_mask.dimension(0) ? normal_mask.dimension(0) - r_n : 4;
        long c_extents = c_n + 4 >= normal_mask.dimension(1) ? normal_mask.dimension(1) - c_n : 4;
        this->interpolation_mask.slice(ar2{r_i, c_i}, ar2{r_extents, c_extents}).setConstant(true);
    }
    this->triangulation.insert(this->points.begin(), this->points.end());
}

float NormalEstimator::getInterpolatedRange(const Triangulation::Point &pt, const Triangulation::Face_handle &fh) {
    double retval = 0;
    double w_sum = 0;
    for(int i = 0; i < 3; ++i) {
        auto &point = fh->mirror_vertex(i)->point();
        float range = fh->mirror_vertex(i)->info();
        double delX = (pt.x() - point.x());
        double delY = (pt.y() - point.y());

        if (std::abs(delY) > 0.174533 || std::abs(delX) > 0.06981) {
            retval = 0;
            w_sum = 1;
            break;
        }

        double dist = std::sqrt(delX * delX + delY * delY);

        double weight = 1.0 / dist;
        retval += range * weight;
        w_sum += weight;
    }
    retval /= w_sum;
    return (float)(retval);
}

void NormalEstimator::interpolate() {
    for (int i = 0; i < this->interpolation_mask.dimension(0); ++i) {
        for (int j = 0; j < this->interpolation_mask.dimension(1); ++j) {
            std::unique_ptr<Triangulation::Face_handle> p_fh(nullptr);
            if (this->interpolation_mask(i,j)) {
                double azimuth = this->starting_azimuth - j*this->param.azimuth_resolution;
                double elevation = this->param.min_elevation + i*this->param.elevation_resolution;
                Triangulation::Point pt(elevation, azimuth);
                Triangulation::Face_handle fh;
                if (p_fh) {
                    fh = this->triangulation.locate(pt, *(p_fh));
                } else {
                    fh = this->triangulation.locate(pt);
                    p_fh = std::make_unique<Triangulation::Face_handle>();
                }
                (*p_fh) = fh;
                this->interpolated_image(i,j) = this->getInterpolatedRange(pt, fh);
            }
        }
    }
}

const Eigen::Tensor<bool, 2>& NormalEstimator::getNormalMask() {
    return this->normal_mask;
}

const Eigen::Tensor<bool, 2>& NormalEstimator::getInterpolationMask() {
    return this->interpolation_mask;
}

const Eigen::Tensor<float, 2>& NormalEstimator::getInterpolatedImage() {
    return this->interpolated_image;
}
}
