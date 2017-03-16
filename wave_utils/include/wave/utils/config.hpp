#ifndef WAVE_UTILS_CONFIG_HPP
#define WAVE_UTILS_CONFIG_HPP

#include <type_traits>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "wave/utils/math.hpp"
#include "wave/utils/logging.hpp"
#include "wave/utils/filesystem.hpp"


namespace wave {

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

    bool *b;
    int *i;
    float *f;
    double *d;
    std::string *s;

    std::vector<bool> *b_array;
    std::vector<int> *i_array;
    std::vector<float> *f_array;
    std::vector<double> *d_array;
    std::vector<std::string> *s_array;

    Vec2 *vec2;
    Vec3 *vec3;
    Vec4 *vec4;
    VecX *vecx;

    Mat2 *mat2;
    Mat3 *mat3;
    Mat4 *mat4;
    MatX *matx;
    cv::Mat *cvmat;

    ConfigParam(void) {
        this->type = TYPE_NOT_SET;
        this->key = "";
        this->optional = false;

        this->b = NULL;
        this->i = NULL;
        this->f = NULL;
        this->d = NULL;
        this->s = NULL;

        this->b_array = NULL;
        this->i_array = NULL;
        this->f_array = NULL;
        this->d_array = NULL;
        this->s_array = NULL;

        this->vec2 = NULL;
        this->vec3 = NULL;
        this->vec4 = NULL;
        this->vecx = NULL;

        this->mat2 = NULL;
        this->mat3 = NULL;
        this->mat4 = NULL;
        this->matx = NULL;
        this->cvmat = NULL;
    }
};

class ConfigParser {
public:
    bool configured;
    bool loaded;

    YAML::Node root;
    std::vector<ConfigParam> params;

    ConfigParser(void);

    template <typename T>
    void addParam(std::string key, T *out, bool optional = false) {
        ConfigParam param;

        // setup
        param.key = key;
        param.optional = optional;

        // primitives
        if (std::is_same<T, bool>::value) {
            param.type = BOOL;
            param.b = reinterpret_cast<bool *>(out);
        } else if (std::is_same<T, int>::value) {
            param.type = INT;
            param.i = reinterpret_cast<int *>(out);
        } else if (std::is_same<T, float>::value) {
            param.type = FLOAT;
            param.f = reinterpret_cast<float *>(out);
        } else if (std::is_same<T, double>::value) {
            param.type = DOUBLE;
            param.d = reinterpret_cast<double *>(out);
        } else if (std::is_same<T, std::string>::value) {
            param.type = STRING;
            param.s = reinterpret_cast<std::string *>(out);
        }

        // arrays
        if (std::is_same<T, std::vector<bool>>::value) {
            param.type = BOOL_ARRAY;
            param.b_array = (std::vector<bool> *) out;
            param.b_array = reinterpret_cast<std::vector<bool> *>(out);
        } else if (std::is_same<T, std::vector<int>>::value) {
            param.type = INT_ARRAY;
            param.i_array = reinterpret_cast<std::vector<int> *>(out);
        } else if (std::is_same<T, std::vector<float>>::value) {
            param.type = FLOAT_ARRAY;
            param.f_array = reinterpret_cast<std::vector<float> *>(out);
        } else if (std::is_same<T, std::vector<double>>::value) {
            param.type = DOUBLE_ARRAY;
            param.d_array = reinterpret_cast<std::vector<double> *>(out);
        } else if (std::is_same<T, std::vector<std::string>>::value) {
            param.type = STRING_ARRAY;
            param.s_array = reinterpret_cast<std::vector<std::string> *>(out);
        }

        // vectors
        if (std::is_same<T, Vec2>::value) {
            param.type = VEC2;
            param.vec2 = reinterpret_cast<Vec2 *>(out);
        } else if (std::is_same<T, Vec3>::value) {
            param.type = VEC3;
            param.vec3 = reinterpret_cast<Vec3 *>(out);
        } else if (std::is_same<T, Vec4>::value) {
            param.type = VEC4;
            param.vec4 = reinterpret_cast<Vec4 *>(out);
        } else if (std::is_same<T, VecX>::value) {
            param.type = VECX;
            param.vecx = reinterpret_cast<VecX *>(out);
        }

        // matrix
        if (std::is_same<T, Mat2>::value) {
            param.type = MAT2;
            param.mat2 = reinterpret_cast<Mat2 *>(out);
        } else if (std::is_same<T, Mat3>::value) {
            param.type = MAT3;
            param.mat3 = reinterpret_cast<Mat3 *>(out);
        } else if (std::is_same<T, Mat4>::value) {
            param.type = MAT4;
            param.mat4 = reinterpret_cast<Mat4 *>(out);
        } else if (std::is_same<T, MatX>::value) {
            param.type = MATX;
            param.matx = reinterpret_cast<MatX *>(out);
        } else if (std::is_same<T, cv::Mat>::value) {
            param.type = CVMAT;
            param.cvmat = reinterpret_cast<cv::Mat *>(out);
        }

        // add params
        if (param.type == TYPE_NOT_SET) {
            LOG_ERROR("TYPE [%s] HAS NOT BEEN IMPLEMENTED YET!",
                      typeid(T).name());
        } else {
            this->params.push_back(param);
        }
    }

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
