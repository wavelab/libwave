/**
 * Class to count bin directions
 */

#ifndef WAVE_ICOSAHEDRON_BINNER_HPP
#define WAVE_ICOSAHEDRON_BINNER_HPP

#include "wave/utils/math.hpp"

namespace wave {

class IcosahedronBinner {
 public:
    IcosahedronBinner() {}
    IcosahedronBinner(uint32_t angular_bins);

    void setAngularBins(uint32_t new_angular_bin);

    /**
     * Zeros out bin counters;
     */
    void clear();

    /**
     * Assigns a vector to a bin, and returns the total count of elements in that bin
     */
    int bin(const Vec6 &unit_vector);

    /**
     * Decrements the bin count
     */
    int deBin(const Vec6 &unit_vector);

    /**
     * Bins unit_vector if the number of elements in that bin is less than or equal to limit
     */
    bool bin(const Vec6 &unit_vector, int limit);

    /**
     *
     * @param unit_vector
     * @return
     */
    void getBinIndex(const Vec6 &unit_vector, uint32_t &angular_bin, uint32_t &dir_bin) const;

    const auto& getBinCounters() {
        return this->bin_counters;
    }

 private:
    std::vector<std::vector<int>> bin_counters;
};

}

#endif //WAVE_ICOSAHEDRON_BINNER_HPP
