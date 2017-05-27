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

template <typename FactorType, typename... Args>
void FactorGraph::addFactor(Args &&... args) {
    this->factors.emplace_back(
      std::make_shared<FactorType>(std::forward<Args>(args)...));
}

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
    int index = 0;

    for (auto f : graph.factors) {
        //        os << "f" << index << " -- ";
        //        os << *f << std::endl;
        index++;
    }

    return os;
}

}  // namespace wave
