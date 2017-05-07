## Defines

    #define E_CSV_DATA_LOAD "Error! failed to load test data [%s]!!\n"
    #define E_CSV_DATA_OPEN "Error! failed to open file for output [%s]!!\n"









## Functions

    int csvrows(std::string file_path)


---

    int mat2csv(std::string file_path,
                MatX data)


---

    int csv2mat(std::string file_path,
                bool header,
                MatX &data)


---

    int csvcols(std::string file_path)


---