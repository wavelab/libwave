#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP

#include <iostream>

#include "wave/utils/utils.hpp"

namespace wave {

using FactorId = size_t;

struct FactorVariable {
    FactorId id;
    VecX data;

    FactorVariable() : id{0}, data{VecX::Zero(1)} {}
    FactorVariable(size_t id, size_t size) : id{id}, data{VecX::Zero(size)} {}

    friend std::ostream &operator<<(std::ostream &os,
                                    const FactorVariable &var) {
        os << "[";
        os << "id: " << var.id << ", ";
        os << "size: " << var.data.size();
        os << "]";
        return os;
    }

    bool operator<(const FactorVariable &other) const {
        if (this->id < other.id) {
            return true;
        }
        return false;
    }
};

struct PoseVar : FactorVariable {
    PoseVar(size_t id) : FactorVariable{id, 6} {}
};

struct LandmarkVar : FactorVariable {
    LandmarkVar(size_t id) : FactorVariable{id, 3} {}
};

}  // end of wave namespace
#endif
