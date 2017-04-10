#include "wave/optimization/factor_graph/factor_graph.hpp"

namespace wave {

FactorGraph::FactorGraph(void) {
    this->variables.clear();
    this->graph.clear();
}

int FactorGraph::addUnaryFactor(size_t at, double measurement) {
    std::pair<size_t, std::shared_ptr<Factor>> graph_value;
    std::shared_ptr<Factor> factor_ptr(nullptr);

    factor_ptr = std::make_shared<Factor>(at, measurement);
    this->variables.insert(at);
    this->factors.push_back(factor_ptr);

    graph_value = std::make_pair(at, factor_ptr);
    this->graph.insert(graph_value);

    return 0;
}

int FactorGraph::addBinaryFactor(size_t from, size_t to, double measurement) {
    std::pair<size_t, std::shared_ptr<Factor>> graph_value;
    std::shared_ptr<Factor> factor_ptr(nullptr);

    factor_ptr = std::make_shared<Factor>(from, to, measurement);
    this->variables.insert(from);
    this->variables.insert(to);
    this->factors.push_back(factor_ptr);

    graph_value = std::make_pair(from, factor_ptr);
    this->graph.insert(graph_value);

    graph_value = std::make_pair(to, factor_ptr);
    this->graph.insert(graph_value);

    return 0;
}

int FactorGraph::print(void) {
    int index = 0;

    for (auto x : this->factors) {
        std::cout << "f" << index << " -- ";

        if (x->arity == 1) {
            std::cout << x->connections[0] << std::endl;
        } else if (x->arity == 2) {
            std::cout << x->connections[0] << ", ";
            std::cout << x->connections[1] << std::endl;
        }

        index++;
    }

    return 0;
}

}  // end of wave namespace
