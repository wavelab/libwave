#include "wave/utils/math.hpp"


namespace wave {

int randi(int ub, int lb) {
    return rand() % lb + ub;
}

double randf(double ub, double lb) {
    double f = (double) rand() / RAND_MAX;
    return lb + f * (ub - lb);
}

int fltcmp(double f1, double f2, double threshold) {
    if (fabs(f1 - f2) <= threshold) {
        return 0;
    } else if (f1 > f2) {
        return 1;
    } else {
        return -1;
    }
}

double median(std::vector<double> v) {
    double a, b;

    // sort values
    std::sort(v.begin(), v.end());

    // obtain median
    if (v.size() % 2 == 1) {
        // return middle value
        return v[v.size() / 2];

    } else {
        // grab middle two values and calc mean
        a = v[v.size() / 2];
        b = v[(v.size() / 2) - 1];
        return (a + b) / 2.0;
    }
}

double deg2rad(double d) {
    return d * (M_PI / 180);
}

double rad2deg(double r) {
    return r * (180 / M_PI);
}

void vec2mat(std::vector<double> x, int rows, int cols, MatX &y) {
    int idx;

    // setup
    idx = 0;
    y.resize(rows, cols);

    // load matrix
    for (int i = 0; i < cols; i++) {
        for (int j = 0; j < rows; j++) {
            y(j, i) = x[idx];
            idx++;
        }
    }
}

void mat2vec(MatX A, std::vector<double> &x) {
    for (int i = 0; i < A.cols(); i++) {
        for (int j = 0; j < A.rows(); j++) {
            x.push_back(A(j, i));
        }
    }
}

double wrapTo180(double euler_angle) {
    return fmod((euler_angle + 180.0), 360.0) - 180.0;
}

double wrapTo360(double euler_angle) {
    if (euler_angle > 0) {
        return fmod(euler_angle, 360.0);
    } else {
        euler_angle += 360.0;
        return fmod(euler_angle, 360.0);
    }
}

int euler2rot(Vec3 euler, int euler_seq, Mat3 &R) {
    double R11, R12, R13, R21, R22, R23, R31, R32, R33;
    double phi, theta, psi;

    phi = euler(0);
    theta = euler(1);
    psi = euler(2);

    if (euler_seq == 321) {
        // euler 3-2-1
        R11 = cos(theta) * cos(psi);
        R12 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
        R13 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);

        R21 = cos(theta) * sin(psi);
        R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
        R23 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);

        R31 = -sin(theta);
        R32 = sin(phi) * cos(theta);
        R33 = cos(phi) * cos(theta);

    } else if (euler_seq == 123) {
        // euler 1-2-3
        R11 = cos(theta) * cos(psi);
        R12 = cos(theta) * sin(psi);
        R13 = -sin(theta);

        R21 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
        R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
        R23 = sin(phi) * cos(theta);

        R31 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
        R32 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
        R33 = cos(phi) * cos(theta);

    } else {
        return -1;
    }

    R << R11, R12, R13, R21, R22, R23, R31, R32, R33;

    return 0;
}


}  // eof wave
