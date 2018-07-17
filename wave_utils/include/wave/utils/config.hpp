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

#include "wave/utils/math.hpp"
#include "wave/utils/log.hpp"
#include "wave/utils/file.hpp"


namespace wave {
/** @addtogroup utils
 *  @{ */

/** Return codes for ConfigParser methods */
enum class ConfigStatus {
    OK = 0,                  ///< Success
    MissingOptionalKey = 1,  ///< key not found, but parameter is optional
    // Note: errors are != ConfigStatus::OK
    FileError = -1,      ///< Success
    KeyError = -2,       ///< key not found, parameter is not optional
    ConversionError = -3 /**< Invalid value for conversion
                           * e.g. wrong-size list for vector, missing
                           * key 'rows', 'cols' or 'data' (for matrix) */
};

/** Base class representing a parameter to be parsed in the yaml file */
struct ConfigParamBase {
    ConfigParamBase(std::string key, bool optional)
        : key{std::move(key)}, optional{optional} {};


    /** Parse the given node as T, and write the value into `destination`.
     * @note no lookup by key is performed; the node must already be correct
     * @throws YAML::BadConversion if node is not convertible to destination
     */
    virtual void load(const YAML::Node &node) const = 0;

    std::string key;  //!< yaml key to parse from
    bool optional;    //!< if true, it is not an error if the key is not found

 protected:
    virtual ~ConfigParamBase() = default;
};

/** Represents a parameter to be parsed in the yaml file */
template <typename T>
class ConfigParam : public ConfigParamBase {
 public:
    ConfigParam(std::string key, T *destination, bool optional = false)
        : ConfigParamBase{std::move(key), optional}, data{destination} {}

    /** Parse the given node as T, and write the value into `destination`.
     * @note no lookup by key is performed; the node must already be correct
     * @throws YAML::BadConversion if node is not convertible to destination
     */
    void load(const YAML::Node &node) const override {
        *(this->data) = node.as<T>();
    }

 protected:
    T *data = nullptr;  //!< where we will store the parsed value
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
    std::vector<std::shared_ptr<ConfigParamBase>> params;

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
    template <typename T>
    void addParam(std::string key, T *out, bool optional = false) {
        auto ptr =
          std::make_shared<ConfigParam<T>>(std::move(key), out, optional);
        this->params.push_back(ptr);
    }

    /** Get yaml node given yaml `key`. The result is assigned to `node` if
     * `key` matches anything in the config file, else `node` is set to `NULL`.
     */
    ConfigStatus getYamlNode(const std::string &key, YAML::Node &node);

    /** Check whether a key is present in the yaml file */
    ConfigStatus checkKey(const std::string &key, bool optional);

    /** Load yaml param primitive, array, vector or matrix. */
    ConfigStatus loadParam(const ConfigParamBase &param);

    /** Load yaml file at `config_file`. */
    ConfigStatus load(const std::string &config_file);
};

/** @} group utils */
}  // namespace wave


namespace YAML {

/** Custom conversion functions for YAML -> Eigen matrix
 *
 * @note We require the yaml node to have three nested keys in order to parse a
 * matrix. They are `rows`, `cols` and `data`.
 *
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
 * Conversions for the following types are included in of wave_utils:
 * - Eigen::Matrix2d
 * - Eigen::Matrix3d
 * - Eigen::Matrix4d
 * - Eigen::MatrixXd
 */
template <typename Scalar, int Rows, int Cols>
struct convert<Eigen::Matrix<Scalar, Rows, Cols>> {
    /** Convert YAML node to Eigen Matrix
     *
     * @throws YAML::InvalidNode if nested keys not found
     * @returns true if conversion successful
     */
    static bool decode(const Node &node,
                       Eigen::Matrix<Scalar, Rows, Cols> &out);
};

/** Custom conversion functions for YAML -> Eigen vector
 *
 * For example:
 * ```yaml
 * some_vector: [1.1, 2.2, 3.3]
 * ```
 *
 * Conversions for the following types are compiled as part of wave_utils:
 * - Eigen::Vector2d
 * - Eigen::Vector3d
 * - Eigen::Vector4d
 * - Eigen::VectorXd
 */
template <typename Scalar, int Rows>
struct convert<Eigen::Matrix<Scalar, Rows, 1>> {
    /** Convert YAML node to Eigen Matrix
     * @returns true if conversion successful
     */
    static bool decode(const Node &node, Eigen::Matrix<Scalar, Rows, 1> &out);
};

}  // namespace YAML

#endif  // WAVE_UTILS_CONFIG_HPP
