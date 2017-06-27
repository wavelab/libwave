namespace wave {

// Constructors
FactorGraph::FactorGraph() {}

// Capacity

FactorGraph::size_type FactorGraph::countFactors() const noexcept {
    return this->factors.size();
}

bool FactorGraph::empty() const noexcept {
    return this->factors.empty();
}

// Modifiers
template <typename FuncType, typename MeasType, typename... VarTypes>
void FactorGraph::addFactor(FuncType f,
                            const MeasType &meas,
                            std::shared_ptr<VarTypes>... variables) {
    using FactorType = Factor<MeasType, VarTypes...>;

    // Give a nice error message if the function type is wrong
    static_assert(
      std::is_same<typename FactorType::FuncType *, FuncType>::value,
      "The given measurement function is of incorrect type");

    this->factors.emplace_back(
      std::make_shared<FactorType>(f, meas, std::move(variables)...));
};

// Iterators

typename FactorGraph::iterator FactorGraph::begin() noexcept {
    return this->factors.begin();
}

typename FactorGraph::iterator FactorGraph::end() noexcept {
    return this->factors.end();
}

typename FactorGraph::const_iterator FactorGraph::begin() const noexcept {
    return this->factors.begin();
}

typename FactorGraph::const_iterator FactorGraph::end() const noexcept {
    return this->factors.end();
}

std::ostream &operator<<(std::ostream &os, const FactorGraph &graph) {
    os << "FactorGraph " << graph.countFactors() << " factors [";
    const auto N = graph.countFactors();
    for (auto i = 0 * N; i < N; ++i) {
        os << *graph.factors[i];
        if (i < N - 1) {
            os << ", ";
        }
    }
    os << "]";

    return os;
}

}  // namespace wave
