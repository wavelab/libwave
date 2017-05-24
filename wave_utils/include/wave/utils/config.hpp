/** @file
 * @ingroup utils
 *
 * Contains a useful `ConfigParser` class to simplify parsing yaml files.
 */

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
/** @addtogroup utils
 *  @{ */

/**
 * An enum used by `ConfigParam` to denote the yaml value type.
 *
 *  Currently we support parsing of the following data types:
 *  - **Primitives**: `bool`, `int`, `float`, `double`, `std::string`
 *  - **Arrays**: `std::vector<bool>`, `std::vector<int>`, `std::vector<float>`,
 * `std::vector<double>`, `std::vector<std::string>`
 *  - **Vectors**: `Eigen::Vector2d`, `Eigen::Vector3d`, `Eigen::Vector4d`,
 * `Eigen::VectorXd`
 *  - **Matrices**: `Eigen::Matrix2d`, `Eigen::Matrix3d`, `Eigen::Matrix4d`,
 * `Eigen::MatrixXd`
 */
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

/** Represents a parameter to be parsed in the yaml file */
class ConfigParam {
 public:
    enum ConfigDataType type = TYPE_NOT_SET;  //!< parameter type (e.g `INT`)
    std::string key;                          //!< yaml key to parse from
    void *data = nullptr;  //!< pointer to variable to store the parameter value
    bool optional = false;

    ConfigParam() {}

    ConfigParam(ConfigDataType type, std::string key, void *out, bool optional)
        : type{type}, key{key}, data{out}, optional{optional} {};
};

/** Parses yaml files for parameters of different types.
 *
 * Currently `ConfigParser` supports parsing the following data types:
 * - **Primitives**: `bool`, `int`, `float`, `double`, `std::string`
 * - **Arrays**: `std::vector<bool>`, `std::vector<int>`, `std::vector<float>`,
 * `std::vector<double>`, `std::vector<std::string>`
 * - **Vectors**: `Eigen::Vector2d`, `Eigen::Vector3d`, `Eigen::Vector4d`,
 * `Eigen::VectorXd`
 * - **Matrices**: `Eigen::Matrix2d`, `Eigen::Matrix3d`, `Eigen::Matrix4d`,
 * `Eigen::MatrixXd`
 *
 * @note For a **matrix** we require the yaml to have three nested keys in order
 * for `ConfigParser` to operate properly. They are `rows`, `cols` and `data`.
 * For example:
 * ```yaml
 * some_matrix:
 *   rows: 3
 *   cols: 3
 *   data: [1.1, 2.2, 3.3,
 *          4.4, 5.5, 6.6,
 *          7.7, 8.8, 9.9]
 * ```
 *
 * For example, consider this yaml file:
 * @include example_params.yaml
 *
 * The code that parses the above is:
 * @include example_config_parser.cpp
 *
 * @todo integrate unit tests into examples (see
 * http://stackoverflow.com/a/16034375/431033)
 */
class ConfigParser {
 public:
    bool config_loaded;

    YAML::Node root;
    std::vector<ConfigParam> params;

    /** Default constructor. By default it sets:
     *
     * - `config_loaded` to `false`
     *
     * This variable is set to `true` once the yaml file is loaded.
     */
    ConfigParser(void);

    /** Use the variations of `addParam` to add parameters you would like to
     * parse from the yaml file, where `key` is the yaml key, `out` is
     * dependent on the type of parameter you want to parse to and an
     * `optional` parameter to define whether `ConfigParser` should fail if the
     * parameter is not found.
     */
    void addParam(std::string key, bool *out, bool optional = false);
    void addParam(std::string key, int *out, bool optional = false);
    void addParam(std::string key, float *out, bool optional = false);
    void addParam(std::string key, double *out, bool optional = false);
    void addParam(std::string key, std::string *out, bool optional = false);
    void addParam(std::string key,
                  std::vector<bool> *out,
                  bool optional = false);
    void addParam(std::string key,
                  std::vector<int> *out,
                  bool optional = false);
    void addParam(std::string key,
                  std::vector<float> *out,
                  bool optional = false);
    void addParam(std::string key,
                  std::vector<double> *out,
                  bool optional = false);
    void addParam(std::string key,
                  std::vector<std::string> *out,
                  bool optional = false);
    void addParam(std::string key, Vec2 *out, bool optional = false);
    void addParam(std::string key, Vec3 *out, bool optional = false);
    void addParam(std::string key, Vec4 *out, bool optional = false);
    void addParam(std::string key, VecX *out, bool optional = false);
    void addParam(std::string key, Mat2 *out, bool optional = false);
    void addParam(std::string key, Mat3 *out, bool optional = false);
    void addParam(std::string key, Mat4 *out, bool optional = false);
    void addParam(std::string key, MatX *out, bool optional = false);
    void addParam(std::string key, cv::Mat *out, bool optional = false);

    /** Get yaml node given yaml `key`. The result is assigned to `node` if
     * `key` matches anything in the config file, else `node` is set to `NULL`.
     */
    int getYamlNode(std::string key, YAML::Node &node);
    /** @return a status code meaning
      * - `0`: On success
      * - `-1`: Config file is not loaded
      * - `-2`: `key` not found in yaml file, parameter is not optional
      * - `-3`: `key` not found in yaml file, parameter is optional
      * - `-4`: Invalid vector (wrong size)
      * - `-5`: Invalid matrix (missing yaml key 'rows', 'cols' or 'data' for
     * matrix)
      */
    /** @return see `getYamlNode` */
    int checkKey(std::string key, bool optional);
    /** @return see `checkKey`
     *
     * @todo refactor return codes into an enum which can be documented
     */
    int checkVector(std::string key, enum ConfigDataType type, bool optional);
    /** @return see `checkKey` */
    int checkMatrix(std::string key, bool optional);
    /** Load yaml param primitive, array, vector or matrix.
     * @return
     * - `0`: On success
     * - `-1`: Config file is not loaded
     * - `-2`: `key` not found in yaml file, parameter is not optional
     * - `-3`: `key` not found in yaml file, parameter is optional
     * - `-4`: Invalid vector (wrong size)
     * - `-5`: Invalid matrix (missing yaml key 'rows', 'cols' or 'data' for
     * matrix)
     * - `-6`: Invalid param type
     */
    int loadPrimitive(ConfigParam param);
    int loadArray(ConfigParam param);
    int loadVector(ConfigParam param);
    int loadMatrix(ConfigParam param);
    /** Load yaml file at `config_file`.
     * @return
     * - `0`: On success
     * - `1`: File not found
     * - `-1`: Config file is not loaded
     * - `-2`: `key` not found in yaml file
     * - `-4`: Invalid vector (wrong size)
     * - `-5`: Invalid matrix (missing yaml key 'rows', 'cols' or 'data' for
     * matrix)
     * - `-6`: Invalid param type
     */
    int load(std::string config_file);
};

/** @} end of group */
}  // end of wave namespace
#endif
