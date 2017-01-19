#include "slam/optimization/testcase.hpp"


namespace slam {

TestRange::TestRange(void)
{
    this->x_min = 0.0;
    this->x_max = 0.0;

    this->y_min = 0.0;
    this->y_max = 0.0;

    this->z_min = 0.0;
    this->z_max = 0.0;
}

TestCase::TestCase(void)
{
    this->configured = false;

}

int TestCase::configure(void)
{
    this->configured = true;

    return 0;
}

void createR(double roll, double pitch, double yaw, Mat3 &R)
{
    double psi;
    double theta;
    double phi;

    // setup
    psi = yaw;
    theta = pitch;
    phi = roll;

    // direction cosine matrix (inertial to body)
    R(0, 0) = cos(theta) * cos(psi);
    R(0, 1) = cos(theta) * sin(psi);
    R(0, 2) = -sin(theta);

    R(1, 0) = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    R(1, 1) = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    R(1, 2) = sin(phi) * cos(theta);

    R(2, 0) = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
    R(2, 1) = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
    R(2, 2) = cos(phi) * cos(theta);
}

void TestCase::createR(double qx, double qy, double qz, double qw, Mat3 &R)
{
    // rotation matrix from quaternion q = (x, y, z, w)
    R(0, 0) = 1.0 - 2.0 * pow(qy, 2) - 2.0 * pow(qz, 2);
    R(0, 1) = 2.0 * qx * qy + 2.0 * qw * qz;
    R(0, 2) = 2.0 * qx * qz - 2.0 * qw * qy;

    R(1, 0) = 2.0 * qx * qy - 2.0 * qw * qz;
    R(1, 1) = 1.0 - 2.0 * pow(qx, 2) - 2.0 * pow(qz, 2);
    R(1, 2) = 2.0 * qy * qz + 2.0 * qw * qz;

    R(2, 0) = 2.0 * qx * qz - 2.0 * qw * qy;
    R(2, 1) = 2.0 * qy * qz - 2.0 * qw * qx;
    R(2, 2) = 1.0 - 2.0 * pow(qx, 2) - 2.0 * pow(qy, 2);
}

void TestCase::createP(Mat3 K, Mat3 R, Vec3 t, MatX &P)
{
    P.resize(3, 4);
    P.block(0, 0, 3, 3) = R;
    P.block(0, 3, 3, 1) = -(R * t);
    P = K * P;
}

void TestCase::generateRandom3DPoints(
    TestRange range,
    int nb_pts,
    slam::MatX &pts
)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> x_dist(range.x_min, range.x_max);
    std::uniform_real_distribution<double> y_dist(range.y_min, range.y_max);
    std::uniform_real_distribution<double> z_dist(range.z_min, range.z_max);

    pts.resize(nb_pts, 3);
    for (int i = 0; i < nb_pts; i++) {
        pts(i, 0) = x_dist(generator);
        pts(i, 1) = y_dist(generator);
        pts(i, 2) = z_dist(generator);
    }
}

void TestCase::project3DTo2D(MatX P, MatX pts_3d, MatX &pts_2d)
{
    Vec2 p;
    Vec3 x;
    Vec4 X;

    // setup
    pts_2d.resize(pts_3d.rows(), 2);

    // project 3D points to 2D image points
    for (int i = 0; i < pts_3d.rows(); i++) {
        // convert 3d point to homogeneous coordinates
        X << pts_3d(i, 0), pts_3d(i, 1), pts_3d(i, 2), 1.0;

        // project 3d point to first image
        x = P * X;

        // convert homogeneous coordinates to 2d image coordinates
        p << x(0) / x(2), x(1) / x(2);

        pts_2d(i, 0) = p(0);
        pts_2d(i, 1) = p(1);
    }
}

void TestCase::project3DTo2D(Mat3 K, Mat3 R, Vec3 t, MatX pts_3d, MatX &pts_2d)
{
    MatX P;

    this->createP(K, R, t, P);
    this->project3DTo2D(P, pts_3d, pts_2d);
}

void TestCase::generateTestCase(
    TestRange range,
    MatX &pts1,
    MatX &pts2,
    MatX &pts3d
)
{
    Mat3 K, R;
    MatX P;
    Vec2 p;
    Vec3 x, x1, x2, t;
    Vec4 X, q;

    // setup
    this->generateRandom3DPoints(range, 10, pts3d);

    // create correspondance points on first image
    K << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    R << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    t << 0.0, 0.0, 0.0;

    this->project3DTo2D(K, R, t, pts3d, pts1);

    // create correspondance points on second image
    q << 0.0, -0.174, 0.0, 0.985;
    q.normalize();
    this->createR(q(0), q(1), q(2), q(3), R);

    t << 1.0, 0.0, 0.0;

    this->project3DTo2D(K, R, t, pts3d, pts2);
}

}  // end of slam namespace
