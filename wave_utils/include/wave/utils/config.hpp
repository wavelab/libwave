#ifndef WAVE_UTILS_CONFIG_HPP
#define WAVE_UTILS_CONFIG_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <type_traits>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "wave/utils/math.hpp"
#include "wave/utils/logging.hpp"
#include "wave/utils/filesystem.hpp"


namespace wave {

#define CONFIG_PARSER_ADD_PARAM(TYPE, KEY, OUT, OPTIONAL) \
    ConfigParam param;                                    \
    param.type = TYPE;                                    \
    param.key = KEY;                                      \
    param.data = OUT;                                     \
    param.optional = OPTIONAL;                            \
    this->params.push_back(param);


enum ConfigDataType {
    TYPE_NOT_SET = 0,
    // PRIMITIVES
    BOOL = 1,
    INT = 2,
    FLOAT = 3,
    DOUBLE = 4,
    STRING = 5,
    // ARRAY
    BOOL_ARRAY = 11,
    INT_ARRAY = 12,
    FLOAT_ARRAY = 13,
    DOUBLE_ARRAY = 14,
    STRING_ARRAY = 15,
    // VECTOR
    VEC2 = 21,
    VEC3 = 22,
    VEC4 = 23,
    VECX = 24,
    // MATRIX
    MAT2 = 31,
    MAT3 = 32,
    MAT4 = 33,
    MATX = 34,
    CVMAT = 35
};

class ConfigParam {
 public:
    enum ConfigDataType type;
    std::string key;
    bool optional;
    void *data;

    ConfigParam(void) {
        this->type = TYPE_NOT_SET;
        this->key = "";
        this->optional = false;
        this->data = NULL;
    }
};

class ConfigParser {
 public:
    bool config_loaded;

    YAML::Node root;
    std::vector<ConfigParam> params;

    ConfigParser(void);
    // clang-format off
    void addParam(std::string key, bool *out, bool optional = false);
    void addParam(std::string key, int *out, bool optional = false);
    void addParam(std::string key, float *out, bool optional = false);
    void addParam(std::string key, double *out, bool optional = false);
    void addParam(std::string key, std::string *out, bool optional = false);
    void addParam(std::string key, std::vector<bool> *out, bool optional = false);
    void addParam(std::string key, std::vector<int> *out, bool optional = false);
    void addParam(std::string key, std::vector<float> *out, bool optional = false);
    void addParam(std::string key, std::vector<double> *out, bool optional = false);
    void addParam(std::string key, std::vector<std::string> *out, bool optional = false);
    void addParam(std::string key, Vec2 *out, bool optional = false);
    void addParam(std::string key, Vec3 *out, bool optional = false);
    void addParam(std::string key, Vec4 *out, bool optional = false);
    void addParam(std::string key, VecX *out, bool optional = false);
    void addParam(std::string key, Mat2 *out, bool optional = false);
    void addParam(std::string key, Mat3 *out, bool optional = false);
    void addParam(std::string key, Mat4 *out, bool optional = false);
    void addParam(std::string key, MatX *out, bool optional = false);
    void addParam(std::string key, cv::Mat *out, bool optional = false);
    // clang-format on
    int getYamlNode(std::string key, YAML::Node &node);
    int checkKey(std::string key, bool optional);
    int checkVector(std::string key, enum ConfigDataType type, bool optional);
    int checkMatrix(std::string key, bool optional);
    int loadPrimitive(ConfigParam param);
    int loadArray(ConfigParam param);
    int loadVector(ConfigParam param);
    int loadMatrix(ConfigParam param);
    int load(std::string config_file);
};

}  // end of wave namespace
#endif
