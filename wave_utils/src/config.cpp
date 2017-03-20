#include "wave/utils/config.hpp"


namespace wave {

ConfigParser::ConfigParser(void) {
    this->configured = false;
    this->loaded = false;
}

int ConfigParser::getYamlNode(std::string key, YAML::Node &node) {
    std::string element;
    std::istringstream iss(key);
    std::vector<YAML::Node> traversal;

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

    this->getYamlNode(key, node);
    if (!node && optional == false) {
        LOG_ERROR("Opps [%s] missing in yaml file!", key.c_str());
        return -1;
    } else if (!node && optional == true) {
        return 1;
    }

    return 0;
}

int ConfigParser::checkVector(std::string key,
                              enum ConfigDataType type,
                              bool optional) {
    int retval;
    int vector_size;

    // check key
    retval = this->checkKey(key, optional);
    if (retval != 0) {
        return retval;
    }

    // clang-format off
    switch (type) {
        case VEC2: vector_size = 2; break;
        case VEC3: vector_size = 3; break;
        case VEC4: vector_size = 4; break;
        default: return 0;
    }
    // clang-format on

    // check number of values
    if (this->root[key].size() != vector_size) {
        // clang-format off
        LOG_ERROR("Vector [%s] should have %d values but config has %d!",
          key.c_str(),
          vector_size,
          (int) this->root[key].size()
        );
        // clang-format on
        return -3;
    }

    return 0;
}

int ConfigParser::checkMatrix(std::string key, bool optional) {
    int retval;
    const std::string targets[3] = {"rows", "cols", "data"};

    // check key
    retval = this->checkKey(key, optional);
    if (retval != 0) {
        return retval;
    }

    // check fields
    for (int i = 0; i < 3; i++) {
        if (!this->root[key][targets[i]]) {
            LOG_ERROR("Key [%s] is missing for matrix [%s]!",
                      targets[i].c_str(),
                      key.c_str());
            return -2;
        }
    }

    return 0;
}

int ConfigParser::loadPrimitive(ConfigParam param) {
    int retval;
    YAML::Node node;

    // pre-check
    retval = this->checkKey(param.key, param.optional);
    if (retval != 0) {
        return retval;
    }

    // parse
    this->getYamlNode(param.key, node);
    // clang-format off
    switch (param.type) {
        case BOOL: *param.b = node.as<bool>(); break;
        case INT: *param.i = node.as<int>(); break;
        case FLOAT: *param.f = node.as<float>(); break;
        case DOUBLE: *param.d = node.as<double>(); break;
        case STRING: *param.s = node.as<std::string>(); break;
    }
    // clang-format on

    return 0;
}

int ConfigParser::loadArray(ConfigParam param) {
    int retval;
    YAML::Node node;

    // pre-check
    retval = this->checkKey(param.key, param.optional);
    if (retval != 0) {
        return retval;
    }

    // parse
    this->getYamlNode(param.key, node);
    switch (param.type) {
        case BOOL_ARRAY:
            for (int i = 0; i < node.size(); i++) {
                param.b_array->push_back(node[i].as<bool>());
            }
            break;
        case INT_ARRAY:
            for (int i = 0; i < node.size(); i++) {
                param.i_array->push_back(node[i].as<int>());
            }
            break;
        case FLOAT_ARRAY:
            for (int i = 0; i < node.size(); i++) {
                param.f_array->push_back(node[i].as<float>());
            }
            break;
        case DOUBLE_ARRAY:
            for (int i = 0; i < node.size(); i++) {
                param.d_array->push_back(node[i].as<double>());
            }
            break;
        case STRING_ARRAY:
            for (int i = 0; i < node.size(); i++) {
                param.s_array->push_back(node[i].as<std::string>());
            }
            break;
    }

    return 0;
}

int ConfigParser::loadVector(ConfigParam param) {
    int retval;
    YAML::Node node;

    // check
    retval = this->checkVector(param.key, param.type, param.optional);
    if (retval != 0) {
        return retval;
    }

    // setup
    Vec2 &vec2 = *param.vec2;
    Vec3 &vec3 = *param.vec3;
    Vec4 &vec4 = *param.vec4;
    VecX &vecx = *param.vecx;

    // parse
    this->getYamlNode(param.key, node);

    switch (param.type) {
        case VEC2:
            vec2(0) = node[0].as<double>();
            vec2(1) = node[1].as<double>();
            break;
        case VEC3:
            vec3(0) = node[0].as<double>();
            vec3(1) = node[1].as<double>();
            vec3(2) = node[2].as<double>();
            break;
        case VEC4:
            vec4(0) = node[0].as<double>();
            vec4(1) = node[1].as<double>();
            vec4(2) = node[2].as<double>();
            vec4(3) = node[3].as<double>();
            break;
        case VECX:
            vecx = VecX((int) node.size());
            for (int i = 0; i < node.size(); i++) {
                vecx(i) = node[i].as<double>();
            }
            break;
    }

    return 0;
}

int ConfigParser::loadMatrix(ConfigParam param) {
    int retval;
    int index;
    int rows;
    int cols;
    YAML::Node node;

    // pre-check
    retval = this->checkMatrix(param.key, param.optional);
    if (retval != 0) {
        return retval;
    }

    // setup
    Mat2 &mat2 = *param.mat2;
    Mat3 &mat3 = *param.mat3;
    Mat4 &mat4 = *param.mat4;
    MatX &matx = *param.matx;
    cv::Mat &cvmat = *param.cvmat;

    // parse
    this->getYamlNode(param.key, node);
    index = 0;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();

    switch (param.type) {
        case MAT2:
            mat2(0, 0) = node["data"][0].as<double>();
            mat2(0, 1) = node["data"][1].as<double>();

            mat2(1, 0) = node["data"][2].as<double>();
            mat2(1, 1) = node["data"][3].as<double>();
            break;
        case MAT3:
            mat3(0, 0) = node["data"][0].as<double>();
            mat3(0, 1) = node["data"][1].as<double>();
            mat3(0, 2) = node["data"][2].as<double>();

            mat3(1, 0) = node["data"][3].as<double>();
            mat3(1, 1) = node["data"][4].as<double>();
            mat3(1, 2) = node["data"][5].as<double>();

            mat3(2, 0) = node["data"][6].as<double>();
            mat3(2, 1) = node["data"][7].as<double>();
            mat3(2, 2) = node["data"][8].as<double>();
            break;
        case MAT4:
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    mat4(i, j) = node["data"][index].as<double>();
                    index++;
                }
            }
            break;
        case MATX:
            matx.resize(rows, cols);
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    matx(i, j) = node["data"][index].as<double>();
                    index++;
                }
            }
            break;
        case CVMAT:
            cvmat = cv::Mat(rows, cols, CV_64F);
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    cvmat.at<double>(i, j) = node["data"][index].as<double>();
                    index++;
                }
            }
            break;
    }

    return 0;
}

int ConfigParser::load(std::string config_file) {
    int retval;

    // pre-check
    if (file_exists(config_file) == false) {
        LOG_ERROR("File not found: %s", config_file.c_str());
        return -1;
    }

    // load and parse file
    this->root = YAML::LoadFile(config_file);

    for (int i = 0; i < this->params.size(); i++) {
        switch (this->params[i].type) {
            // PRIMITIVE
            case BOOL:
            case INT:
            case FLOAT:
            case DOUBLE:
            case STRING:
                retval = this->loadPrimitive(this->params[i]);
                break;
            // ARRAY
            case BOOL_ARRAY:
            case INT_ARRAY:
            case FLOAT_ARRAY:
            case DOUBLE_ARRAY:
            case STRING_ARRAY:
                retval = this->loadArray(this->params[i]);
                break;
            // VECTOR
            case VEC2:
            case VEC3:
            case VEC4:
            case VECX:
                retval = this->loadVector(this->params[i]);
                break;
            // MAT
            case MAT2:
            case MAT3:
            case MAT4:
            case MATX:
            case CVMAT:
                retval = this->loadMatrix(this->params[i]);
                break;
        }

        if (retval != 0) {
            return retval;
        }
    }

    return 0;
}

}  // end of wave namespace
