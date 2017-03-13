#include "wave/utils/math.hpp"


namespace wave {

int randi(int ub, int lb) {
  return rand() % lb + ub;
}

double randf(double ub, double lb) {
  double f = (double) rand() / RAND_MAX;
  return lb + f * (ub - lb);
}

int fltcmp(double f1, double f2) {
  if (fabs(f1 - f2) <= 0.0001) {
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

void load_matrix(std::vector<double> x, int rows, int cols, MatX &y) {
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

void load_matrix(MatX A, std::vector<double> &x) {
  for (int i = 0; i < A.cols(); i++) {
    for (int j = 0; j < A.rows(); j++) {
      x.push_back(A(j, i));
    }
  }
}

int euler2quat(Vec3 euler, int euler_seq, Quaternion &q) {
  double alpha, beta, gamma;
  double c1, c2, c3, s1, s2, s3;
  double w, x, y, z;

  alpha = euler(0);
  beta = euler(1);
  gamma = euler(2);

  c1 = cos(alpha / 2.0);
  c2 = cos(beta / 2.0);
  c3 = cos(gamma / 2.0);
  s1 = sin(alpha / 2.0);
  s2 = sin(beta / 2.0);
  s3 = sin(gamma / 2.0);

  switch (euler_seq) {
    case 123:
      // euler 1-2-3 to quaternion
      w = c1 * c2 * c3 - s1 * s2 * s3;
      x = s1 * c2 * c3 + c1 * s2 * s3;
      y = c1 * s2 * c3 - s1 * c2 * s3;
      z = c1 * c2 * s3 + s1 * s2 * c3;
      break;

    case 321:
      // euler 3-2-1 to quaternion
      w = c1 * c2 * c3 + s1 * s2 * s3;
      x = s1 * c2 * c3 - c1 * s2 * s3;
      y = c1 * s2 * c3 + s1 * c2 * s3;
      z = c1 * c2 * s3 - s1 * s2 * c3;
      break;

    default:
      printf("Error! Invalid euler sequence [%d]\n", euler_seq);
      return -1;
      break;
  }

  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;

  return 0;
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

int quat2euler(Quaternion q, int euler_seq, Vec3 &euler) {
  double qw, qx, qy, qz;
  double qw2, qx2, qy2, qz2;
  double phi, theta, psi;

  qw = q.w();
  qx = q.x();
  qy = q.y();
  qz = q.z();

  qw2 = pow(qw, 2);
  qx2 = pow(qx, 2);
  qy2 = pow(qy, 2);
  qz2 = pow(qz, 2);

  if (euler_seq == 123) {
    // euler 1-2-3
    phi = atan2(2 * (qz * qw - qx * qy), (qw2 + qx2 - qy2 - qz2));
    theta = asin(2 * (qx * qz + qy * qw));
    psi = atan2(2 * (qx * qw - qy * qz), (qw2 - qx2 - qy2 + qz2));

  } else if (euler_seq == 321) {
    // euler 3-2-1
    phi = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
    theta = asin(2 * (qy * qw - qx * qz));
    psi = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  } else {
    return -1;
  }

  euler << phi, theta, psi;
  return 0;
}

int quat2rot(Quaternion q, Mat3 &R) {
  double qw, qx, qy, qz;
  double qw2, qx2, qy2, qz2;
  double R11, R12, R13, R21, R22, R23, R31, R32, R33;

  qw = q.w();
  qx = q.x();
  qy = q.y();
  qz = q.z();

  qw2 = pow(qw, 2);
  qx2 = pow(qx, 2);
  qy2 = pow(qy, 2);
  qz2 = pow(qz, 2);

  // inhomogeneous form
  R11 = 1 - 2 * qy2 - 2 * qz2;
  R12 = 2 * qx * qy + 2 * qz * qw;
  R13 = 2 * qx * qz - 2 * qy * qw;

  R21 = 2 * qx * qy - 2 * qz * qw;
  R22 = 1 - 2 * qx2 - 2 * qz2;
  R23 = 2 * qy * qz + 2 * qx * qw;

  R31 = 2 * qx * qz + 2 * qy * qw;
  R32 = 2 * qy * qz - 2 * qx * qw;
  R33 = 1 - 2 * qx2 - 2 * qy2;

  // // homogeneous form
  // R11 = qx2 + qx2 - qy2 - qz2;
  // R12 = 2 * (qx * qy - qw * qz);
  // R13 = 2 * (qw * qy + qx * qz);
  //
  // R21 = 2 * (qx * qy + qw * qz);
  // R22 = qw2 - qx2 + qy2 - qz2;
  // R23 = 2 * (qy * qz - qw * qx);
  //
  // R31 = 2 * (qx * qz - qw * qy);
  // R32 = 2 * (qw * qx + qy * qz);
  // R33 = qw2 - qx2 - qy2 + qz2;

  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;

  return 0;
}

void enu2nwu(Vec3 enu, Vec3 &nwu) {
  // ENU frame:  (x - right, y - forward, z - up)
  // NWU frame:  (x - forward, y - left, z - up)
  nwu(0) = enu(1);
  nwu(1) = -enu(0);
  nwu(2) = enu(2);
}

void cf2nwu(Vec3 cf, Vec3 &nwu) {
  // camera frame:  (x - right, y - down, z - forward)
  // NWU frame:  (x - forward, y - left, z - up)
  nwu(0) = cf(2);
  nwu(1) = -cf(0);
  nwu(2) = -cf(1);
}

void cf2enu(Vec3 cf, Vec3 &enu) {
  // camera frame:  (x - right, y - down, z - forward)
  // ENU frame:  (x - right, y - forward, z - up)
  enu(0) = cf(0);
  enu(1) = cf(2);
  enu(2) = -cf(1);
}

void nwu2enu(Vec3 nwu, Vec3 &enu) {
  // NWU frame:  (x - forward, y - left, z - up)
  // ENU frame:  (x - right, y - forward, z - up)
  enu(0) = -nwu(1);
  enu(1) = nwu(0);
  enu(2) = nwu(2);
}

void ned2enu(Vec3 ned, Vec3 &enu) {
  // NED frame:  (x - forward, y - right, z - down)
  // ENU frame:  (x - right, y - forward, z - up)
  enu(0) = ned(1);
  enu(1) = ned(0);
  enu(2) = -ned(2);
}

void nwu2ned(Quaternion nwu, Quaternion &ned) {
  ned.w() = nwu.w();
  ned.x() = nwu.x();
  ned.y() = -nwu.y();
  ned.z() = -nwu.z();
}

void ned2nwu(Quaternion ned, Quaternion &nwu) {
  nwu.w() = ned.w();
  nwu.x() = ned.x();
  nwu.y() = -ned.y();
  nwu.z() = -ned.z();
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

double cross_track_error(Vec2 p1, Vec2 p2, Vec2 pos) {
  double x0, y0;
  double x1, y1;
  double x2, y2;
  double numerator, denominator;

  // setup
  x0 = pos(0);
  y0 = pos(1);

  x1 = p1(0);
  y1 = p1(0);

  x2 = p2(0);
  y2 = p2(0);

  // calculate perpendicular distance between line (p1, p2) and point (pos)
  numerator = ((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
  denominator = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
  return fabs(numerator) / denominator;
}

}  // eof wave
