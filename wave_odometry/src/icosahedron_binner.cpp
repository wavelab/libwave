#include "wave/odometry/icosahedron_binner.hpp"

namespace wave {

IcosahedronBinner::IcosahedronBinner() {
    this->bin_counters.resize(static_cast<unsigned long>(11));
    this->clear();
}

void IcosahedronBinner::clear() {
    std::fill(this->bin_counters.begin(), this->bin_counters.end(), 0);
}

int IcosahedronBinner::bin(const wave::Vec3 &unit_vector) {
    auto index = this->getBinIndex(unit_vector);

    return ++(this->bin_counters.at(index));
}

bool IcosahedronBinner::bin(const wave::Vec3 &unit_vector, const int limit) {
    auto index = this->getBinIndex(unit_vector);

    if (this->bin_counters.at(index) < limit) {
        this->bin_counters.at(index)++;
        return true;
    }
    return false;
}

unsigned long IcosahedronBinner::getBinIndex(const wave::Vec3 &unit_vector) {
    //binning anything less than 45 degrees to the vertical
    if (unit_vector(2) > 0.5253) {
        return 0;
    }

    double azimuth = std::atan2(unit_vector(1), unit_vector(0)) + M_PI;

    return static_cast<unsigned long>((azimuth / (2 * M_PI)) * 10.0);
}

}
