#include "wave/utils/config.hpp"


namespace wave {

ConfigParser::ConfigParser(void) {
    this->config_loaded = false;
}

ConfigStatus ConfigParser::getYamlNode(const std::string &key,
                                       YAML::Node &node) {
    std::string element;
    std::istringstream iss(key);
    std::vector<YAML::Node> traversal;

    // pre-check
    if (this->config_loaded == false) {
        return ConfigStatus::FileError;
    }

    // recurse down config key
    traversal.push_back(this->root);
    while (std::getline(iss, element, '.')) {
        traversal.push_back(traversal.back()[element]);
    }
    node = traversal.back();
    // Note:
    //
    //    yaml_node = yaml_node["some_level_deeper"];
    //
    // YAML::Node is mutable, by doing the above it destroys the parsed yaml
    // tree/graph, to avoid this problem we store the visited YAML::Node into
    // a std::vector and return the last visited YAML::Node

    return ConfigStatus::OK;
}

ConfigStatus ConfigParser::checkKey(const std::string &key, bool optional) {
    YAML::Node node;

    // pre-check
    if (this->config_loaded == false) {
        return ConfigStatus::FileError;
    }

    // check key
    this->getYamlNode(key, node);
    if (!node && optional == false) {
        LOG_ERROR("[%s] missing in yaml file!", key.c_str());
        return ConfigStatus::KeyError;
    } else if (!node && optional == true) {
        return ConfigStatus::MissingOptionalKey;
    }

    return ConfigStatus::OK;
}

ConfigStatus ConfigParser::loadParam(const ConfigParamBase &param) {
    YAML::Node node;

    // Check that the key exists
    const auto retval = this->checkKey(param.key, param.optional);
    if (retval != ConfigStatus::OK) {
        return retval;
    }

    this->getYamlNode(param.key, node);

    // Attempt to parse
    try {
        param.load(node);
    } catch (const YAML::RepresentationException &e) {
        // The convert() function for the value type returned false
        return ConfigStatus::ConversionError;
    }

    return ConfigStatus::OK;
}

ConfigStatus ConfigParser::load(const std::string &config_file) {
    // pre-check
    if (file_exists(config_file) == false) {
        LOG_ERROR("File not found: %s", config_file.c_str());
        return ConfigStatus::FileError;
    }

    // load and parse file
    this->root = YAML::LoadFile(config_file);
    this->config_loaded = true;

    for (const auto &param_ptr : this->params) {
        const auto retval = this->loadParam(*param_ptr);
        if (retval != ConfigStatus::OK) {
            return retval;
        }
    }

    return ConfigStatus::OK;
}

}  // namespace wave


namespace YAML {

template <typename Scalar, int Rows, int Cols>
bool convert<Eigen::Matrix<Scalar, Rows, Cols>>::decode(
  const Node &node, Eigen::Matrix<Scalar, Rows, Cols> &out) {
    int rows = node["rows"].as<int>();
    int cols = node["cols"].as<int>();
    const auto &data = node["data"];

    // Check `rows` and `cols` values
    if ((rows != Rows && Rows != Eigen::Dynamic) ||
        (cols != Cols && Cols != Eigen::Dynamic)) {
        return false;
    }

    // Check data node is a list of the right length
    std::size_t expected_size = rows * cols;
    if (!data.IsSequence() || data.size() != expected_size) {
        return false;
    }

    // Copy it to destination
    // In case it's dynamic, resize (no effect on fixed)
    out.resize(rows, cols);
    for (int i = 0, index = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            out(i, j) = data[index++].as<double>();
        }
    }
    return true;
}

template <typename Scalar, int Rows>
bool convert<Eigen::Matrix<Scalar, Rows, 1>>::decode(
  const Node &node, Eigen::Matrix<Scalar, Rows, 1> &out) {
    int rows = node.size();

    // Check data node is a list of the right length
    if (!node.IsSequence()) {
        return false;
    }
    // For fixed-size destination, yaml list must match
    if (rows != Rows && Rows != Eigen::Dynamic) {
        return false;
    }

    // In case it's dynamic, resize (no effect on fixed)
    out.resize(rows);

    // Copy it to destination
    for (int i = 0, index = 0; i < rows; i++) {
        out(i) = node[index++].as<double>();
    }
    return true;
}

// Explicit instantiations: include these in the compiled wave_utils library
// Since the function definition is in a .cpp file, other types will not work
template struct convert<wave::Mat2>;
template struct convert<wave::Mat3>;
template struct convert<wave::Mat4>;
template struct convert<wave::Mat6>;
template struct convert<wave::MatX>;

template struct convert<wave::Vec2>;
template struct convert<wave::Vec3>;
template struct convert<wave::Vec4>;
template struct convert<wave::VecX>;

template struct convert<wave::Mat2f>;
template struct convert<wave::Mat3f>;
template struct convert<wave::Mat4f>;
template struct convert<wave::Mat6f>;
template struct convert<wave::MatXf>;

}  // namespace YAML
