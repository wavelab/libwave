#include "wave/odometry/icosahedron_binner.hpp"

namespace wave {

IcosahedronBinner::IcosahedronBinner(IcosahedronBinnerParams params) {
    this->setParams(params);
}

void IcosahedronBinner::setParams(const IcosahedronBinnerParams &new_param) {
    this->params = new_param;
    auto range_bins = this->params.range_divisions.size() + 1;
    auto dirs = this->params.xy_directions + 1;
    this->bin_counters = Eigen::Tensor<int, 3>(dirs, this->params.azimuth_divisions, range_bins);
    this->clear();
}

void IcosahedronBinner::clear() {
    this->bin_counters.setZero();
}

bool IcosahedronBinner::bin(const wave::Vec6 &unit_vector, int *bin_count) {
    uint32_t range_bin, angular_bin, dir_bin;
    this->getBinIndex(unit_vector, range_bin, angular_bin, dir_bin);

    bool retval = true;

    // separate limit for z direction
    int n_limit;
    if (dir_bin == 0) {
        n_limit = this->params.z_limit;
    } else {
        n_limit = this->params.xy_limit;
    }
    if (this->bin_counters(dir_bin, angular_bin, range_bin) < n_limit) {
        this->bin_counters(dir_bin, angular_bin, range_bin)++;
    } else {
        retval = false;
    }

    if (bin_count) {
        *bin_count = this->bin_counters(dir_bin, angular_bin, range_bin);
    }

    return retval;
}

bool IcosahedronBinner::spaceInBin(const wave::Vec6 &unit_vector) {
    uint32_t range_bin, angular_bin, dir_bin;
    this->getBinIndex(unit_vector, range_bin, angular_bin, dir_bin);

    // separate limit for z direction
    int n_limit;
    if (dir_bin == 0) {
        n_limit = this->params.z_limit;
    } else {
        n_limit = this->params.xy_limit;
    }
    if (this->bin_counters(dir_bin, angular_bin, range_bin) < n_limit) {
        return true;
    }
    return false;
}

int IcosahedronBinner::deBin(const wave::Vec6 &unit_vector) {
    uint32_t range_bin, angular_bin, dir_bin = 0;
    this->getBinIndex(unit_vector, range_bin, range_bin, angular_bin);

    if (this->bin_counters(dir_bin, angular_bin, range_bin) > 0) {
        this->bin_counters(dir_bin, angular_bin, range_bin)--;
    }
    return this->bin_counters(dir_bin, angular_bin, range_bin);
}

void IcosahedronBinner::getBinIndex(const wave::Vec6 &unit_vector, uint32_t &range_bin, uint32_t &angular_bin,
                                    uint32_t &dir_bin) const {
    if (unit_vector(2) > this->params.z_cutoff) {
        dir_bin = 0;
    } else {
        double dir_az = std::atan2(unit_vector(1), unit_vector(0)) + M_PI;
        dir_bin = static_cast<uint32_t>((dir_az / (2 * M_PI)) * this->params.xy_directions) + 1;
    }

    double pt_az = std::atan2(unit_vector(4), unit_vector(3)) + M_PI;
    angular_bin = static_cast<uint32_t>((pt_az / (2 * M_PI)) * (double)(this->params.azimuth_divisions));

    double range = unit_vector.block<3,1>(3,0).norm();
    auto iter = std::upper_bound(this->params.range_divisions.begin(), this->params.range_divisions.end(), range);

    if (iter == this->params.range_divisions.end()) {
        range_bin = static_cast<uint32_t> (this->params.range_divisions.size());
    } else {
        range_bin = static_cast<uint32_t>(iter - this->params.range_divisions.begin());
    }
}

}
