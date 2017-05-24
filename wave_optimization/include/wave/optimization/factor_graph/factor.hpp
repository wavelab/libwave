#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP

#include <memory>

#include "wave/utils/utils.hpp"
#include "wave/optimization/factor_graph/variable.hpp"

namespace wave {

struct Factor {
    const std::vector<std::shared_ptr<FactorVariable>> variables;
    MatX measurement;

    Factor() : variables{}, measurement{MatX::Zero(1, 1)} {}

    Factor(const std::shared_ptr<FactorVariable> &at, const MatX &measurement)
        : variables{at}, measurement{measurement} {}

    Factor(const std::shared_ptr<FactorVariable> &from,
           const std::shared_ptr<FactorVariable> &to,
           const MatX &measurement)
        : variables{from, to}, measurement{measurement} {}

    friend std::ostream &operator<<(std::ostream &os, const Factor &factor) {
        os << "[";

        os << "arity: " << factor.variables.size() << ", ";

        os << "variables: ";
        for (size_t i = 0; i < factor.variables.size(); i++) {
            if (i != 0) {
                os << ", ";
            }
            os << *factor.variables[i];
        }

        os << "]";
        return os;
    }
};

struct UnaryFactor : Factor {
    UnaryFactor(const std::shared_ptr<FactorVariable> &at,
                const MatX &measurement)
        : Factor{at, measurement} {}
};

struct BinaryFactor : Factor {
    BinaryFactor(const std::shared_ptr<FactorVariable> &from,
                 const std::shared_ptr<FactorVariable> &to,
                 const MatX &measurement)
        : Factor{from, to, measurement} {}
};

}  // end of wave namespace
#endif
