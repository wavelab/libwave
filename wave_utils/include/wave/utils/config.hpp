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

#include "wave/utils/math.hpp"
#include "wave/utils/log.hpp"
#include "wave/utils/file.hpp"


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
    ~ConfigParamBase() = default;  // disallow deletion through pointer to base
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
        auto ptr = std::make_shared<ConfigParam<T>>(key, out, optional);
        this->params.push_back(ptr);
    }

    /** Get yaml node given yaml `key`. The result is assigned to `node` if
     * `key` matches anything in the config file, else `node` is set to `NULL`.
     */
    int getYamlNode(std::string key, YAML::Node &node);

    /** @return a status code meaning
      * - `0`: On success
      * - `-1`: Config file is not loaded
      * - `-2`: `key` not found in yaml file, parameter is not optional
      * - `-3`: `key` not found in yaml file, parameter is optional
      * - `-4`: Invalid value for conversion: e.g. wrong size list (for vector),
      *   missing yaml key 'rows', 'cols' or 'data' (for matrix)
      *
      * @todo refactor return codes into an enum which can be documented
      */
    int checkKey(std::string key, bool optional);

    /** Load yaml param primitive, array, vector or matrix.
     * @return
     * - `0`: On success
     * - `-1`: Config file is not loaded
     * - `-2`: `key` not found in yaml file, parameter is not optional
     * - `-3`: `key` not found in yaml file, parameter is optional
     * - `-4`: Invalid value for conversion: e.g. wrong size list (for vector),
     *   missing yaml key 'rows', 'cols' or 'data' (for matrix)
     * - `-6`: Invalid param type
     */
    int loadParam(const ConfigParamBase &param);

    /** Load yaml file at `config_file`.
     * @return
     * - `0`: On success
     * - `1`: File not found
     * - `-1`: Config file is not loaded
     * - `-2`: `key` not found in yaml file
     * - `-4`: Invalid value for conversion: e.g. wrong size list (for vector),
     *   missing yaml key 'rows', 'cols' or 'data' (for matrix)
     * - `-6`: Invalid param type
     */
    int load(std::string config_file);
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
 */
template <typename Scalar, int Rows, int Cols>
struct convert<Eigen::Matrix<Scalar, Rows, Cols>> {
    static Node encode(const Eigen::Matrix<Scalar, Rows, Cols> &) {
        throw std::logic_error("Writing matrix to yaml not implemented");
    }

    /** Convert YAML node to Eigen Matrix
     *
     * @throws YAML::InvalidNode if nested keys not found
     * @returns true if conversion successful
     */
    static bool decode(const Node &node,
                       Eigen::Matrix<Scalar, Rows, Cols> &out) {
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
};

/** Custom conversion functions for YAML -> Eigen vector
 *
 * For example:
 * ```yaml
 * some_vector: [1.1, 2.2, 3.3]
 * ```
 */
template <typename Scalar, int Rows>
struct convert<Eigen::Matrix<Scalar, Rows, 1>> {
    static Node encode(const Eigen::Matrix<Scalar, Rows, 1> &) {
        throw std::logic_error("Writing vector to yaml not implemented");
    }

    /** Convert YAML node to Eigen Matrix
     * @returns true if conversion successful
     */
    static bool decode(const Node &node, Eigen::Matrix<Scalar, Rows, 1> &out) {
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
};

/** Custom conversion functions for YAML -> cv::Mat
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

 */
template <>
struct convert<cv::Mat> {
    static Node encode(const cv::Mat &) {
        throw std::logic_error("Writing matrix to yaml not implemented");
    }

    /** Convert YAML node to cv matrix
     *
     * @throws YAML::InvalidNode if nested keys not found
     * @returns true if conversion successful
     */
    static bool decode(const Node &node, cv::Mat &out) {
        int rows = node["rows"].as<int>();
        int cols = node["cols"].as<int>();
        const auto &data = node["data"];

        // Check `rows` and `cols` values
        if (rows <= 0 || cols <= 0) {
            return false;
        }

        // Check data node is a list of the right length
        std::size_t expected_size = rows * cols;
        if (!data.IsSequence() || data.size() != expected_size) {
            return false;
        }

        // Copy it to destination
        out = cv::Mat{rows, cols, CV_64F};
        for (int i = 0, index = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                out.at<double>(i, j) = node["data"][index++].as<double>();
            }
        }
        return true;
    }
};

}  // namespace YAML

#endif  // WAVE_UTILS_CONFIG_HPP
