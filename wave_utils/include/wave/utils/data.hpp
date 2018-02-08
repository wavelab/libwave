/** @file
 * @ingroup utils
 *
 * Functions to load / save matrix data in csv format.
 */

#ifndef WAVE_UTILS_DATA_HPP
#define WAVE_UTILS_DATA_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <matplotlibcpp.h>

#include "wave/utils/math.hpp"
#include "wave/utils/log.hpp"

namespace wave {
/** @addtogroup utils
 *  @{ */

#define PYTHON_SCRIPT(A)                                                       \
  if (system("python " A) != 0) {                                             \
    LOG_ERROR("Python script [%s] failed !", A);                               \
    return -1;                                                                 \
  }

// CSV ERROR MESSAGES
#define E_CSV_DATA_LOAD "Error! failed to load test data [%s]!!\n"
#define E_CSV_DATA_OPEN "Error! failed to open file for output [%s]!!\n"

/** @return  the number of rows in the csv file at `file_path`, or `-1` if the
 * function failed to open the csv file. */
int csvrows(std::string file_path);

/** @return  the number of columns in the csv file at `file_path`, or `-1` if
 * the
 * function failed to open the csv file. */
int csvcols(std::string file_path);

/** Load csv file containing a matrix.
 *
 * The parsed matrix will be loaded to `data`.
 *
 * @param file_path path to the csv file
 * @param header whether a header line exists in the csv file.
 * @param[out] data
 *
 * @return `0` on success, `-1` on error
 *
 */
int csv2mat(std::string file_path, bool header, MatX &data);

/** Saves matrix to file.
 *
 * The `data` is saved in csv format to a file at `file_path`.
 * @return `0` on success, `-1` on error
 */
int mat2csv(std::string file_path, MatX data);

/** Plots a vector
 */

template<typename Derived>
void plotVec(const Eigen::MatrixBase<Derived> &mat, bool block = false) {
    Eigen::Map<const Eigen::RowVectorXd> vec(mat.derived().data(), mat.derived().size());

    std::vector<double> xvals;
    std::vector<double> yvals;
    xvals.resize(vec.size());
    yvals.resize(vec.size());
    for (uint32_t i = 0; i < vec.size(); i++) {
        xvals.at(i) = (double) i;
        yvals.at(i) = vec(i);
    }

    matplotlibcpp::plot(xvals, yvals);
    matplotlibcpp::show(block);
}

template<typename Derived>
int plotMat(const Eigen::MatrixBase<Derived> &mat) {
    mat2csv("/tmp/tmp.mat", mat);
    PYTHON_SCRIPT("/home/bapskiko/git/libwave/wave_utils/scripts/plot_matrix.py /tmp/tmp.mat");
    return 0;
}

/** Reads a matrix from an input stream.
 *
 * The entries must be separated by whitespace, and be in row-major order.
 */
template <int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> matrixFromStream(std::istream &in_stream) {
    Eigen::Matrix<double, Rows, Cols> mat;
    for (auto i = 0; i < Rows; ++i) {
        for (auto j = 0; j < Cols; ++j) {
            in_stream >> mat(i, j);
        }
    }

    return mat;
}

/** @} group utils */
}  // namespace wave

#endif  // WAVE_UTILS_DATA_HPP
