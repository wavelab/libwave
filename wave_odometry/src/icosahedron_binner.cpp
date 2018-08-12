#include "wave/odometry/icosahedron_binner.hpp"

namespace wave {

IcosahedronBinner::IcosahedronBinner() {
    this->bin_vectors.resize(3, 12);

    const double gr = (1.0 + std::sqrt(5)) / 2.0;
    const double gr2 = gr * gr;
    const double gr_2 = 1.0 / gr2;

    this->bin_vectors.col(0) << 0.0, 0.0, 1.0;
    this->bin_vectors.col(1) = -this->bin_vectors.col(0);

    this->bin_vectors.col(2) << 2.0 / std::sqrt(5), 0.0, -(2.0 /(gr2 + 1.0) - 1);
    this->bin_vectors.col(3) = -this->bin_vectors.col(2);

    this->bin_vectors.col(4) << 1.0 / (gr2 + 1.0), 1.0 / std::sqrt(gr_2 + 1.0), 1.0 / sqrt(5.0);
    this->bin_vectors.col(5) = -this->bin_vectors.col(4);

    this->bin_vectors.col(6) << 1.0 / (gr2 + 1.0), -1.0 / std::sqrt(gr_2 + 1.0), 1.0 / sqrt(5.0);
    this->bin_vectors.col(7) = -this->bin_vectors.col(6);

    this->bin_vectors.col(8) << -1.0 / std::sqrt(gr_2 + 1.0), -1.0 / (gr2 + 1.0), 1.0 / sqrt(5.0);
    this->bin_vectors.col(9) = -this->bin_vectors.col(8);

    this->bin_vectors.col(10) << -1.0 / std::sqrt(gr_2 + 1.0), 1.0 / (gr2 + 1.0), 1.0 / sqrt(5.0);
    this->bin_vectors.col(11) = -this->bin_vectors.col(10);

    this->nn_search = Nabo::NNSearchD::createKDTreeLinearHeap(this->bin_vectors);

    this->bin_counters.resize(static_cast<unsigned long>(this->bin_vectors.cols()));
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
    Eigen::MatrixXi index_mat(1,1);
    Eigen::MatrixXd distance_mat(1,1);
    this->nn_search->knn(unit_vector, index_mat, distance_mat);

    return static_cast<unsigned long>(index_mat(0));
}

}
