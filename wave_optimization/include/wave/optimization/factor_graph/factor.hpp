#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP

#include <memory>

#include "wave/utils/utils.hpp"
#include "wave/optimization/factor_graph/variable.hpp"

namespace wave {

struct Factor {
    const int arity;
    const std::vector<std::shared_ptr<FactorVariable>> variables;

    Factor() : arity{0}, variables{} {}

    Factor(const std::shared_ptr<FactorVariable> &at)
        : arity{1}, variables{at} {}

    Factor(const std::shared_ptr<FactorVariable> &from,
           const std::shared_ptr<FactorVariable> &to)
        : arity{2}, variables{from, to} {}

    friend std::ostream &operator<<(std::ostream &os, const Factor &factor) {
        os << "[";
        os << "arity: " << factor.arity << ", ";

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

template <typename T>
struct UnaryFactor : Factor {
    T measurement;

    UnaryFactor(const std::shared_ptr<FactorVariable> &at, const T &measurement)
        : Factor{at}, measurement{measurement} {}
};

template <typename T>
struct BinaryFactor : Factor {
    T measurement;

    BinaryFactor(const std::shared_ptr<FactorVariable> &from,
                 const std::shared_ptr<FactorVariable> &to,
                 const T &measurement)
        : Factor{from, to}, measurement{measurement} {}
};

}  // end of wave namespace
#endif
