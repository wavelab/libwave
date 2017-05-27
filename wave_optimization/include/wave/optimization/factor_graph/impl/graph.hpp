namespace wave {

// Constructors
FactorGraph::FactorGraph() {}

// Capacity

FactorGraph::size_type FactorGraph::countVariables() const noexcept {
    return this->variables.size();
}

FactorGraph::size_type FactorGraph::countFactors() const noexcept {
    return this->factors.size();
}

/** Return total number of factors and variables */
FactorGraph::size_type FactorGraph::size() const noexcept {
    return this->countVariables() + this->countFactors();
}

bool FactorGraph::empty() const noexcept {
    // Note: can't have factors without variables
    return this->variables.empty();
}

// Modifiers

template <typename FactorType, typename... Args>
void FactorGraph::addFactor(Args &&... args) {
    auto factor = std::make_shared<FactorType>(std::forward<Args>(args)...);

    this->factors.push_back(factor);
    //    this->variables.insert({key, var});
}

std::ostream &operator<<(std::ostream &os, const FactorGraph &graph) {
    int index = 0;

    for (auto f : graph.factors) {
        //        os << "f" << index << " -- ";
        //        os << *f << std::endl;
        index++;
    }

    return os;
}

}  // namespace wave
