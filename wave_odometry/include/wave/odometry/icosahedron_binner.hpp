/**
 * Class for incremental binning of unit vectors. There are 12 bins.
 * It is possible to extend this class for finer binning, but this application does not require it.
 * https://www.researchgate.net/publication/258712786_Analysis_of_binning_of_normals_for_spherical_harmonic_cross-correlation
 */

#ifndef WAVE_ICOSAHEDRON_BINNER_HPP
#define WAVE_ICOSAHEDRON_BINNER_HPP

#include "wave/utils/math.hpp"
#include <nabo/nabo.h>

namespace wave {

class IcosahedronBinner {
 public:
    IcosahedronBinner();

    /*
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

 private:
    unsigned long getBinIndex(const Vec3 &unit_vector);

    MatX bin_vectors;
    std::vector<int> bin_counters;

    Nabo::NNSearchD* nn_search;
};

}

#endif //WAVE_ICOSAHEDRON_BINNER_HPP
