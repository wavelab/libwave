#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP

namespace wave {

struct Variable {
    size_t id;
    size_t size;

    Variable() : id{0}, size{0} {}
    Variable(size_t id, size_t size) : id{id}, size{size} {}

    std::string toString() {
        std::ostringstream oss;

        oss << "[";
        oss << "id: " << this->id << ", ";
        oss << "size: " << this->size;
        oss << "]";

        return oss.str();
    }

    void print() {
        std::cout << this->toString() << std::endl;
    }

    bool operator<(const Variable &other) const {
        if (this->id < other.id) {
            return true;
        }
        return false;
    }
};

struct PoseVar : Variable {
    PoseVar(size_t id) : Variable{id, 6} {}
};

struct LandmarkVar : Variable {
    LandmarkVar(size_t id) : Variable{id, 3} {}
};

}  // end of wave namespace
#endif
