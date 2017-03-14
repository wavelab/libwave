#ifndef __WAVE_UTILS_DATA_HPP__
#define __WAVE_UTILS_DATA_HPP__

#include <iostream>
#include <fstream>
#include <vector>

#include "wave/utils/math.hpp"


namespace wave {

// CSV ERROR MESSAGES
#define E_CSV_DATA_LOAD "Error! failed to load test data [%s]!!\n"
#define E_CSV_DATA_OPEN "Error! failed to open file for output [%s]!!\n"

class Pose {
public:
    Vec3 position;
    Quaternion orientation;

    Pose(void);
    Pose(Vec3 position, Quaternion orientation);
    Pose(double roll, double pitch, double yaw, double x, double y, double z);
    Mat3 rotationMatrix(void);
    void printPosition(void);
    void printOrientation(void);
    void print(void);
};

class Velocity {
public:
    double linear_x;
    double linear_y;
    double linear_z;

    double angular_x;
    double angular_y;
    double angular_z;

    Velocity(void) {
        this->linear_x = 0;
        this->linear_y = 0;
        this->linear_z = 0;
        this->angular_x = 0;
        this->angular_y = 0;
        this->angular_z = 0;
    }
};

int csvrows(std::string file_path);
int csvcols(std::string file_path);
int csv2mat(std::string file_path, bool header, MatX &data);
int mat2csv(std::string file_path, MatX data);

}  // end of wave namespace
#endif
