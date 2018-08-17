#include "wave/odometry/icosahedron_binner.hpp"

namespace wave {

IcosahedronBinner::IcosahedronBinner(uint32_t angular_bins) {
    this->setAngularBins(angular_bins);
}

void IcosahedronBinner::setAngularBins(uint32_t new_angular_bin) {
    this->bin_counters.resize(new_angular_bin);
    for (auto &counter : this->bin_counters) {
        counter.resize(11);
    }
    this->clear();
}

void IcosahedronBinner::clear() {
    for (auto &counter : this->bin_counters) {
        std::fill(counter.begin(), counter.end(), 0);
    }
}

int IcosahedronBinner::bin(const wave::Vec6 &unit_vector) {
    uint32_t angular_bin, dir_bin;
    this->getBinIndex(unit_vector, angular_bin, dir_bin);

    this->bin_counters.at(angular_bin).at(dir_bin)++;

    return this->bin_counters.at(angular_bin).at(dir_bin);
}

int IcosahedronBinner::deBin(const wave::Vec6 &unit_vector) {
    uint32_t angular_bin, dir_bin;
    this->getBinIndex(unit_vector, angular_bin, dir_bin);

    if (this->bin_counters.at(angular_bin).at(dir_bin) == 0) {
        return 0;
    }
    this->bin_counters.at(angular_bin).at(dir_bin)--;
    return this->bin_counters.at(angular_bin).at(dir_bin);
}

bool IcosahedronBinner::bin(const wave::Vec6 &unit_vector, const int limit) {
    uint32_t angular_bin, dir_bin;
    this->getBinIndex(unit_vector, angular_bin, dir_bin);

    if (this->bin_counters.at(angular_bin).at(dir_bin) < limit) {
        this->bin_counters.at(angular_bin).at(dir_bin)++;
        return true;
    }
    return false;
}

void IcosahedronBinner::getBinIndex(const wave::Vec6 &unit_vector, uint32_t &angular_bin, uint32_t &dir_bin) const {
    //binning anything less than 45 degrees to the vertical
    if (unit_vector(2) > 0.5253) {
        dir_bin = 0;
    } else {
        double dir_az = std::atan2(unit_vector(1), unit_vector(0)) + M_PI;
        dir_bin = static_cast<uint32_t>((dir_az / (2 * M_PI)) * 10.0);
    }

    double pt_az = std::atan2(unit_vector(4), unit_vector(3)) + M_PI;
    angular_bin = static_cast<uint32_t>((pt_az / (2 * M_PI)) * (double)(this->bin_counters.size()));
}

}
