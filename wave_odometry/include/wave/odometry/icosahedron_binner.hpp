/**
 * Class to count bin directions
 */

#ifndef WAVE_ICOSAHEDRON_BINNER_HPP
#define WAVE_ICOSAHEDRON_BINNER_HPP

#include "wave/utils/math.hpp"
#include <unsupported/Eigen/CXX11/Tensor>

namespace wave {

struct IcosahedronBinnerParams {
    std::vector<double> range_divisions;
    int azimuth_divisions;
    int xy_directions;
    double z_cutoff;
    int z_limit;
    int xy_limit;
};

class IcosahedronBinner {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IcosahedronBinner() {}
    IcosahedronBinner(IcosahedronBinnerParams params);

    void setParams(const IcosahedronBinnerParams &new_param);

    /**
     * Zeros out bin counters;
     */
    void clear();

    /**
     * Assigns a vector to a bin if there is still space, and returns the total count of elements in that bin
     * 
     * returns true if the bin counter was increased, false if not
     */
    bool bin(const wave::Vec6 &unit_vector, int *bin_count = nullptr);

    /**
     * Checks whether there is space to bin the given vector
     * DOES NOT increase bin counters
     * returns true if there is space, false if not
     */
    bool spaceInBin(const wave::Vec6 &unit_vector);

    /**
     * Decrements the bin count
     */
    int deBin(const Vec6 &unit_vector);

    /**
     *
     * @param unit_vector
     * @return
     */
    void getBinIndex(const wave::Vec6 &unit_vector, uint32_t &range_bin, uint32_t &angular_bin,
                     uint32_t &dir_bin) const;

    const auto& getBinCounters() {
        return this->bin_counters;
    }

 private:
    Eigen::Tensor<int, 3> bin_counters;
    IcosahedronBinnerParams params;
};

}

#endif //WAVE_ICOSAHEDRON_BINNER_HPP
