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

#include "wave/utils/math.hpp"


namespace wave {
/** @addtogroup utils
 *  @{ */

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

/** @} end of group */
}  // namespace wave

#endif  // WAVE_UTILS_DATA_HPP
