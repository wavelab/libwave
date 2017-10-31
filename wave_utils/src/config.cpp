#include "wave/utils/config.hpp"


namespace wave {

ConfigParser::ConfigParser(void) {
    this->config_loaded = false;
}


int ConfigParser::getYamlNode(std::string key, YAML::Node &node) {
    std::string element;
    std::istringstream iss(key);
    std::vector<YAML::Node> traversal;

    // pre-check
    if (this->config_loaded == false) {
        return -1;
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

    return 0;
}

int ConfigParser::checkKey(std::string key, bool optional) {
    YAML::Node node;

    // pre-check
    if (this->config_loaded == false) {
        return -1;
    }

    // check key
    this->getYamlNode(key, node);
    if (!node && optional == false) {
        LOG_ERROR("Opps [%s] missing in yaml file!", key.c_str());
        return -2;
    } else if (!node && optional == true) {
        return -3;
    }

    return 0;
}

int ConfigParser::loadParam(const ConfigParamBase &param) {
    int retval;
    YAML::Node node;

    // Check that the key exists
    retval = this->checkKey(param.key, param.optional);
    if (retval != 0) {
        return retval;
    }

    this->getYamlNode(param.key, node);

    // Attempt to parse
    try {
        param.load(node);
    } catch (const YAML::RepresentationException &e) {
        // The convert() function for the value type returned false
        return -4;
    }

    return 0;
}

int ConfigParser::load(std::string config_file) {
    int retval;

    // pre-check
    if (file_exists(config_file) == false) {
        LOG_ERROR("File not found: %s", config_file.c_str());
        return 1;
    }

    // load and parse file
    this->root = YAML::LoadFile(config_file);
    this->config_loaded = true;

    for (const auto &param_ptr : this->params) {
        retval = this->loadParam(*param_ptr);
        if (retval != 0 && retval != -3) {
            // Error, return immediately
            return retval;
        }
    }

    return 0;
}

}  // namespace wave
