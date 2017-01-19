#include "slam/vision/eight_point.hpp"


namespace slam {
namespace optimization {

EightPoint::EightPoint(void)
{
    this->configured = false;

    this->image_width = 0;
    this->image_height = 0;
    this->N << 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0,
               0.0, 0.0, 0.0;
}

int EightPoint::configure(int image_width, int image_height)
{
    this->configured = true;

    this->image_width = image_width;
    this->image_height = image_height;
    this->N << 2.0 / image_width, 0.0, -1.0,
               0.0, 2.0 / image_height, -1.0,
               0.0, 0.0, 1.0;

    return 0;
}

void EightPoint::normalizePoints(MatX &pts1, MatX &pts2)
{
    pts1 = (this->N * pts1.transpose()).transpose();
    pts2 = (this->N * pts2.transpose()).transpose();
}

void EightPoint::formMatrixA(MatX &pts1, MatX &pts2, MatX &A)
{
    int rows;
    VecX x1, x2, y1, y2, ones;

    // setup
    rows = pts1.rows();
    x1 = pts1.block(0, 0, rows, 1);
    x2 = pts2.block(0, 0, rows, 1);
    y1 = pts1.block(0, 1, rows, 1);
    y2 = pts2.block(0, 1, rows, 1);
    ones = MatX::Ones(rows, 1);

    // form matrix A; Af = 0
    A.resize(rows, 9);
    A.block(0, 0, rows, 1) = x1.cwiseProduct(x2);
    A.block(0, 1, rows, 1) = y1.cwiseProduct(x2);
    A.block(0, 2, rows, 1) = x2;
    A.block(0, 3, rows, 1) = x1.cwiseProduct(y2);
    A.block(0, 4, rows, 1) = y1.cwiseProduct(y2);
    A.block(0, 5, rows, 1) = y2;
    A.block(0, 6, rows, 1) = x1;
    A.block(0, 7, rows, 1) = y1;
    A.block(0, 8, rows, 1) = ones;
}

void EightPoint::approximateFundamentalMatrix(MatX &A, MatX &F)
{
    MatX U, V;
    Eigen::JacobiSVD<MatX> svd;

    // compute SVD of A
    svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = -svd.matrixU();
    V = -svd.matrixV();

    // form approximate matrix F by
    // extracting 9th-col of V (9 x 1) to form F (3 x 3)
    F = V.block(0, 8, 9, 1);
    F.resize(3, 3);
    F.transposeInPlace();
}

void EightPoint::refineFundamentalMatrix(MatX &F)
{
    Mat3 D;
    MatX U, V, S;
    Eigen::JacobiSVD<MatX> svd;

    // recompute SVD using approximate matrix F
    svd.compute(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();

    D = MatX::Zero(3, 3);
    D(0, 0) = S(0);
    D(1, 1) = S(1);
    D(2, 2) = 0;
    F = U * D * V.transpose();
}

void EightPoint::denormalizeFundamentalMatrix(MatX &F)
{
    F = this->N.transpose() * F * this->N;
}

int EightPoint::estimate(MatX pts1, MatX pts2, MatX &F)
{
    VecX S;
    MatX A;

    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (pts1.size() != pts2.size()) {
        return -2;
    }

    // calculate fundamental matrix
    this->normalizePoints(pts1, pts2);
    this->formMatrixA(pts1, pts2, A);
    this->approximateFundamentalMatrix(A, F);
    this->refineFundamentalMatrix(F);
    this->denormalizeFundamentalMatrix(F);

    return 0;
}

int EightPoint::estimate(MatX pts1, MatX pts2, Mat3 &K, Mat3 &E)
{
    VecX S;
    MatX A, F;

    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (pts1.size() != pts2.size()) {
        return -2;
    }

    // calculate fundamental matrix
    this->normalizePoints(pts1, pts2);
    this->formMatrixA(pts1, pts2, A);
    this->approximateFundamentalMatrix(A, F);
    this->refineFundamentalMatrix(F);
    this->denormalizeFundamentalMatrix(F);

    // convert fundamental to essential matrix E
    E = K.transpose() * F * K;

    return 0;
}

int EightPoint::obtainPossiblePoses(Mat3 E, std::vector<MatX> &poses)
{
    Mat3 W;
    MatX U, V;
    Vec3 T1, T2;
    MatX R1, R2, P1, P2, P3, P4;
    Eigen::JacobiSVD<MatX> svd;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // setup
    W << 0.0, -1.0, 0.0,
         1.0, 0.0, 0.0,
         0.0, 0.0, 1.0;
    P1 = MatX::Zero(3, 4);
    P2 = MatX::Zero(3, 4);
    P3 = MatX::Zero(3, 4);
    P4 = MatX::Zero(3, 4);

    // compute SVD of E
    svd.compute(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();

    // construct 4 possible poses from essential matrix E
    R1 = U * W * V.transpose();
    R2 = U * W.transpose() * V.transpose();
    T1 = U.block(0, 2, 3, 1);
    T2 = -U.block(0, 2, 3, 1);

    // lambda = 1
    P1.block(0, 0, 3, 3) = R1;
    P1.block(0, 3, 3, 1) = T1;
    poses.push_back(P1);

    P2.block(0, 0, 3, 3) = R2;
    P2.block(0, 3, 3, 1) = T1;
    poses.push_back(P2);

    // lambda = -1
    P3.block(0, 0, 3, 3) = R1;
    P3.block(0, 3, 3, 1) = T2;
    poses.push_back(P3);

    P4.block(0, 0, 3, 3) = R2;
    P4.block(0, 3, 3, 1) = T2;
    poses.push_back(P4);

    return 0;
}

int EightPoint::obtainPose(
    Vec3 pt1,
    Vec3 pt2,
    Mat3 K1,
    Mat3 K2,
    std::vector<MatX> poses,
    MatX &pose
)
{
    MatX P1(3, 4), P2(3, 4);
    Mat4 A, V;
    Vec4 p;
    Vec3 x;
    float out_norm, w, T, d1, d2;
    Eigen::JacobiSVD<MatX> svd;

    // first camera matrix P1 (set as origin)
    P1 << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0;

    // normalize pt1 and pt2
    pt1 = K1.inverse() * pt1;
    pt2 = K2.inverse() * pt2;

    // for each camera matrix P, reproject the pair of points in 3D
    for (int i = 0; i < 4; i++) {
        // second camera matrix P2
        P2 = poses[i];

        // build matrix A
        A << (pt1(0) * P1.block(2, 0, 1, 4)) - P1.block(0, 0, 1, 4),
             (pt1(1) * P1.block(2, 0, 1, 4)) - P1.block(1, 0, 1, 4),
             (pt2(0) * P2.block(2, 0, 1, 4)) - P2.block(0, 0, 1, 4),
             (pt2(1) * P2.block(2, 0, 1, 4)) - P2.block(1, 0, 1, 4);

        // normalize A - row by row
        A.block(0, 0, 1, 4) = A.block(0, 0, 1, 4) / A.block(0, 0, 1, 4).norm();
        A.block(1, 0, 1, 4) = A.block(1, 0, 1, 4) / A.block(1, 0, 1, 4).norm();
        A.block(2, 0, 1, 4) = A.block(2, 0, 1, 4) / A.block(2, 0, 1, 4).norm();
        A.block(3, 0, 1, 4) = A.block(3, 0, 1, 4) / A.block(3, 0, 1, 4).norm();

        // compute AX = 0 with SVD to obtain 3D point coordinates
        svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        V = svd.matrixV();
        p = V.block(0, 3, 4, 1);

        // compute depth of 3D point from camera 1
        x = P1 * p;
        w = x(2);
        T = p(3);
        out_norm = P1.block(2, 0, 1, 3).norm();
        d1 = slam::sign(P1.block(0, 0, 3, 3).determinant()) * w / T * out_norm;

        // compute depth of 3D point from camera 2
        x = P2 * p;
        w = x(2);
        T = p(3);
        out_norm = P2.block(2, 0, 1, 3).norm();
        d2 = slam::sign(P2.block(0, 0, 3, 3).determinant()) * w / T * out_norm;

        // return pose if both depths are non-zero
        if (d1 > 0 && d2 > 0) {
            pose = P2;
        }
    }

    return 0;
}


}  // end of optimization namespace
}  // end of slam namespace
