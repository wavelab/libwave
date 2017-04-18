#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP

namespace wave {

enum class VariableType { NOT_SET = 0, POSE = 1, LANDMARK = 2 };

struct Variable {
    VariableType type;
    size_t id;

    Variable() : type(VariableType::NOT_SET), id{0} {}
    Variable(VariableType type, size_t id) : type{type}, id{id} {}

    std::string toString() {
        std::ostringstream oss;

        oss << "[";
        oss << "id: " << this->id << ", ";
        oss << "var_type: ";
        switch (this->type) {
            case VariableType::POSE: oss << "POSE"; break;
            case VariableType::LANDMARK: oss << "LANDMARK"; break;
            default: oss << "NOT_SET"; break;
        }
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
    PoseVar(size_t id) : Variable{VariableType::POSE, id} {}
};

struct LandmarkVar : Variable {
    LandmarkVar(size_t id) : Variable{VariableType::LANDMARK, id} {}
};

}  // end of wave namespace
#endif
