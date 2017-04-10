#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_GRAPH_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_GRAPH_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <memory>

namespace wave {

enum MType {
    DOUBLE = 1,
    VEC2 = 2,
    VEC3 = 3,
    VEC4 = 4,
    VECX = 5,
    MAT2 = 6,
    MAT3 = 7,
    MAT4 = 8,
    MAT5 = 9
};

class Factor {
 public:
    int arity;
    int connections[2];
    enum MType type;
    void *measurement;

    Factor(size_t at, double measurement) {
        this->arity = 1;
        this->connections[0] = at;
        this->connections[1] = -1;
        this->type = DOUBLE;
        this->measurement = new double(measurement);
    }

    Factor(size_t from, size_t to, double measurement) {
        this->arity = 2;
        this->connections[0] = from;
        this->connections[1] = to;
        this->type = DOUBLE;
        this->measurement = new double(measurement);
    }

    ~Factor(void) {
        switch (this->type) {
            case DOUBLE: delete static_cast<double *>(this->measurement); break;
            default: break;
        }
        this->measurement = NULL;
    }
};

class FactorGraph {
 public:
    std::multimap<size_t, std::shared_ptr<Factor>> graph;
    std::set<size_t> variables;
    std::vector<std::shared_ptr<Factor>> factors;

    FactorGraph(void);
    int addUnaryFactor(size_t at, double measurement);
    int addBinaryFactor(size_t from, size_t to, double measurement);
    int print(void);
};

}  // end of wave namespace
#endif
