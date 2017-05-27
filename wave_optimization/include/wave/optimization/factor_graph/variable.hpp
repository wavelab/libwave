#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP

#include <iostream>

#include "wave/utils/utils.hpp"

namespace wave {

using VariableId = size_t;

struct FactorVariable {
    VariableId id;
    VecX data;

    FactorVariable() : id{0}, data{VecX::Zero(1)} {}
    FactorVariable(VariableId id, size_t size)
        : id{id}, data{VecX::Zero(size)} {}

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
    PoseVar(VariableId id) : FactorVariable{id, 6} {}
};

struct LandmarkVar : FactorVariable {
    LandmarkVar(VariableId id) : FactorVariable{id, 3} {}
};


struct Landmark2DVar {
    Landmark2DVar() = default;
    explicit Landmark2DVar(const double const *data) : position{data} {}

    Eigen::Map<const Vec2> position{0};

    constexpr static int SizeAtCompileTime = 2;
};

using Vec1 = Eigen::Matrix<double, 1, 1>;

struct Pose2DVar {
    Pose2DVar() = default;

    explicit Pose2DVar(const double const *data)
        : position{data}, orientation{data + 2} {}

    Eigen::Map<const Vec2> position{0};
    Eigen::Map<const Vec1> orientation{0};
    constexpr static int SizeAtCompileTime = 3;
};


}  // end of wave namespace
#endif
