# wave/utils/data.hpp

This module contains functions to load / save matrix data in csv format.


## Functions

    int csvrows(std::string file_path);
    int csvcols(std::string file_path);

Reads csv file at `file_path` where `csvrows()` and `csvcols()` returns the number of rows and columns respectively. If the function failed to open the csv file the function will return `-1`.

---

    int csv2mat(std::string file_path, bool header, MatX &data);

Load csv file containing a matrix located at `file_path`, `header` denotes whether a header line exists in the csv file. The parsed matrix will be loaded to `data`. On success the function will return `0` else if an error occured a return value of `-1` will be given.

---

    int mat2csv(std::string file_path, MatX data);

Save `data` matrix to file in csv format to location defined by `file_path`. On success the function will return `0` else if an error occured a return value of `-1` will be given.
