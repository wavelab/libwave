/**
 * Class to count bin directions
 */

#ifndef WAVE_ICOSAHEDRON_BINNER_HPP
#define WAVE_ICOSAHEDRON_BINNER_HPP

#include "wave/utils/math.hpp"

namespace wave {

class IcosahedronBinner {
 public:
    IcosahedronBinner();

    /**
     * Zeros out bin counters;
     */
    void clear();

    /**
     * Assigns a vector to a bin, and returns the total count of elements in that bin
     */
    int bin(const Vec3 &unit_vector);

    /**
     * Bins unit_vector if the number of elements in that bin is less than or equal to limit
     */
    bool bin(const Vec3 &unit_vector, const int limit);

    const std::vector<int>& getBinCounters() {
        return this->bin_counters;
    }

 private:
    unsigned long getBinIndex(const Vec3 &unit_vector);

    std::vector<int> bin_counters;
};

}

#endif //WAVE_ICOSAHEDRON_BINNER_HPP
