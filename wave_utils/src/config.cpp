#include "wave/utils/config.hpp"


namespace wave {

ConfigParser::ConfigParser(void) {
    this->config_loaded = false;
}

// clang-format off
void ConfigParser::addParam(std::string key, bool *out, bool optional) {
    this->params.emplace_back(BOOL, key, out, optional);
}

void ConfigParser::addParam(std::string key, int *out, bool optional) {
    this->params.emplace_back(INT, key, out, optional);
}

void ConfigParser::addParam(std::string key, float *out, bool optional) {
    this->params.emplace_back(FLOAT, key, out, optional);
}

void ConfigParser::addParam(std::string key, double *out, bool optional) {
    this->params.emplace_back(DOUBLE, key, out, optional);
}

void ConfigParser::addParam(std::string key, std::string *out, bool optional) {
    this->params.emplace_back(STRING, key, out, optional);
}

void ConfigParser::addParam(std::string key, std::vector<bool> *out, bool optional) {
    this->params.emplace_back(BOOL_ARRAY, key, out, optional);
}

void ConfigParser::addParam(std::string key, std::vector<int> *out, bool optional) {
    this->params.emplace_back(INT_ARRAY, key, out, optional);
}

void ConfigParser::addParam(std::string key, std::vector<float> *out, bool optional) {
    this->params.emplace_back(FLOAT_ARRAY, key, out, optional);
}

void ConfigParser::addParam(std::string key, std::vector<double> *out, bool optional) {
    this->params.emplace_back(DOUBLE_ARRAY, key, out, optional);
}

void ConfigParser::addParam(std::string key, std::vector<std::string> *out, bool optional) {
    this->params.emplace_back(STRING_ARRAY, key, out, optional);
}

void ConfigParser::addParam(std::string key, Vec2 *out, bool optional) {
    this->params.emplace_back(VEC2, key, out, optional);
}

void ConfigParser::addParam(std::string key, Vec3 *out, bool optional) {
    this->params.emplace_back(VEC3, key, out, optional);
}

void ConfigParser::addParam(std::string key, Vec4 *out, bool optional) {
    this->params.emplace_back(VEC4, key, out, optional);
}

void ConfigParser::addParam(std::string key, VecX *out, bool optional) {
    this->params.emplace_back(VECX, key, out, optional);
}

void ConfigParser::addParam(std::string key, Mat2 *out, bool optional) {
    this->params.emplace_back(MAT2, key, out, optional);
}

void ConfigParser::addParam(std::string key, Mat3 *out, bool optional) {
    this->params.emplace_back(MAT3, key, out, optional);
}

void ConfigParser::addParam(std::string key, Mat4 *out, bool optional) {
    this->params.emplace_back(MAT4, key, out, optional);
}

void ConfigParser::addParam(std::string key, MatX *out, bool optional) {
    this->params.emplace_back(MATX, key, out, optional);
}

void ConfigParser::addParam(std::string key, cv::Mat *out, bool optional) {
    this->params.emplace_back(CVMAT, key, out, optional);
}
// clang-format on

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

int ConfigParser::checkVector(std::string key,
                              enum ConfigDataType type,
                              bool optional) {
    int retval;
    int vector_size;

    // pre-check
    if (this->config_loaded == false) {
        return -1;
    }

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
    if (this->root[key].size() != static_cast<size_t>(vector_size)) {
        // clang-format off
        LOG_ERROR("Vector [%s] should have %d values but config has %d!",
          key.c_str(),
          vector_size,
          static_cast<int>(this->root[key].size())
        );
        // clang-format on
        return -4;
    }

    return 0;
}

int ConfigParser::checkMatrix(std::string key, bool optional) {
    int retval;
    const std::string targets[3] = {"rows", "cols", "data"};

    // pre-check
    if (this->config_loaded == false) {
        return -1;
    }

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
            return -5;
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
        case BOOL: *(bool *) param.data = node.as<bool>(); break;
        case INT: *(int *) param.data = node.as<int>(); break;
        case FLOAT: *(float *) param.data = node.as<float>(); break;
        case DOUBLE: *(double *) param.data = node.as<double>(); break;
        case STRING: *(std::string *) param.data = node.as<std::string>(); break;
        default: return -6;
    }
    // clang-format on

    return 0;
}

int ConfigParser::loadArray(ConfigParam param) {
    int retval;
    YAML::Node node;

    // pre-check
    if (this->config_loaded == false) {
        return -1;
    }

    // check parameters
    retval = this->checkKey(param.key, param.optional);
    if (retval != 0) {
        return retval;
    }

    // parse
    this->getYamlNode(param.key, node);
    switch (param.type) {
        case BOOL_ARRAY:
            for (auto n : node) {
                ((std::vector<bool> *) param.data)->push_back(n.as<bool>());
            }
            break;
        case INT_ARRAY:
            for (auto n : node) {
                ((std::vector<int> *) param.data)->push_back(n.as<int>());
            }
            break;
        case FLOAT_ARRAY:
            for (auto n : node) {
                ((std::vector<float> *) param.data)->push_back(n.as<float>());
            }
            break;
        case DOUBLE_ARRAY:
            for (auto n : node) {
                ((std::vector<double> *) param.data)
                  ->push_back(n.as<double>());
            }
            break;
        case STRING_ARRAY:
            for (auto n : node) {
                ((std::vector<std::string> *) param.data)
                  ->push_back(n.as<std::string>());
            }
            break;
        default:
            return -6;
    }

    return 0;
}

int ConfigParser::loadVector(ConfigParam param) {
    int retval;
    YAML::Node node;

    // pre-check
    if (this->config_loaded == false) {
        return -1;
    }

    // check parameter
    retval = this->checkVector(param.key, param.type, param.optional);
    if (retval != 0) {
        return retval;
    }

    // parse
    this->getYamlNode(param.key, node);
    // clang-format off
    switch (param.type) {
        case VEC2:
            *((Vec2 *) param.data) << node[0].as<double>(),
                                      node[1].as<double>();
            break;

        case VEC3:
            *((Vec3 *) param.data) << node[0].as<double>(),
                                      node[1].as<double>(),
                                      node[2].as<double>();
            break;

        case VEC4:
            *((Vec4 *) param.data) << node[0].as<double>(),
                                      node[1].as<double>(),
                                      node[2].as<double>(),
                                      node[3].as<double>();
            break;

        case VECX: {
                VecX &vecx = *(VecX *) param.data;
                vecx = VecX((int) node.size());
                for (size_t i = 0; i < node.size(); i++) {
                    vecx(i) = node[i].as<double>();
                }
        } break;

        default:
            return -6;
    }
    // clang-format on

    return 0;
}

int ConfigParser::loadMatrix(ConfigParam param) {
    int retval;
    int index;
    int rows;
    int cols;
    YAML::Node node;

    // pre-check
    if (this->config_loaded == false) {
        return -1;
    }

    // check parameter
    retval = this->checkMatrix(param.key, param.optional);
    if (retval != 0) {
        return retval;
    }

    // parse
    this->getYamlNode(param.key, node);
    index = 0;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();

    switch (param.type) {
        case MAT2: {
            Mat2 &mat2 = *(Mat2 *) param.data;
            mat2(0, 0) = node["data"][0].as<double>();
            mat2(0, 1) = node["data"][1].as<double>();

            mat2(1, 0) = node["data"][2].as<double>();
            mat2(1, 1) = node["data"][3].as<double>();
        } break;

        case MAT3: {
            Mat3 &mat3 = *(Mat3 *) param.data;
            mat3(0, 0) = node["data"][0].as<double>();
            mat3(0, 1) = node["data"][1].as<double>();
            mat3(0, 2) = node["data"][2].as<double>();

            mat3(1, 0) = node["data"][3].as<double>();
            mat3(1, 1) = node["data"][4].as<double>();
            mat3(1, 2) = node["data"][5].as<double>();

            mat3(2, 0) = node["data"][6].as<double>();
            mat3(2, 1) = node["data"][7].as<double>();
            mat3(2, 2) = node["data"][8].as<double>();
        } break;

        case MAT4: {
            Mat4 &mat4 = *(Mat4 *) param.data;
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    mat4(i, j) = node["data"][index].as<double>();
                    index++;
                }
            }
        } break;

        case MATX: {
            MatX &matx = *(MatX *) param.data;
            matx.resize(rows, cols);
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    matx(i, j) = node["data"][index].as<double>();
                    index++;
                }
            }
        } break;

        case CVMAT: {
            cv::Mat &cvmat = *(cv::Mat *) param.data;
            cvmat = cv::Mat(rows, cols, CV_64F);
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    cvmat.at<double>(i, j) = node["data"][index].as<double>();
                    index++;
                }
            }
        } break;
        default:
            return -6;
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

    for (size_t i = 0; i < this->params.size(); i++) {
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
            default:
                return -6;
        }

        if (retval != 0 && retval != -3) {
            return retval;
        }
    }

    return 0;
}

}  // end of wave namespace
