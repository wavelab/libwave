#include "wave/optimization/ceres/point_to_point_residual.hpp"
#include "wave/geometry_og/transformation.hpp"

namespace wave {

bool AnalyticalPointToPoint::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    // modelled as P1 = T*P2
    // r = T*P2 - P1;
    // parameters in order:
    // R11 R21 R31 R12 R22 R32 R13 R23 R33 X Y Z
    residuals[0] = parameters[0][0] * this->P2[0] + parameters[0][3] * this->P2[1] + parameters[0][6] * this->P2[2] + parameters[0][9] - this->P1[0];
    residuals[1] = parameters[0][1] * this->P2[0] + parameters[0][4] * this->P2[1] + parameters[0][7] * this->P2[2] + parameters[0][10] - this->P1[1];
    residuals[2] = parameters[0][2] * this->P2[0] + parameters[0][5] * this->P2[1] + parameters[0][8] * this->P2[2] + parameters[0][11] - this->P1[2];

    if((jacobians != NULL) && (jacobians[0] != NULL)) {
        jacobians[0][0] = this->P2[0];
        jacobians[0][1] = 0;
        jacobians[0][2] = 0;
        jacobians[0][3] = this->P2[1];
        jacobians[0][4] = 0;
        jacobians[0][5] = 0;
        jacobians[0][6] = this->P2[2];
        jacobians[0][7] = 0;
        jacobians[0][8] = 0;
        jacobians[0][9] = 1;
        jacobians[0][10] = 0;
        jacobians[0][11] = 0;

        jacobians[0][12] = 0;
        jacobians[0][13] = this->P2[0];
        jacobians[0][14] = 0;
        jacobians[0][15] = 0;
        jacobians[0][16] = this->P2[1];
        jacobians[0][17] = 0;
        jacobians[0][18] = 0;
        jacobians[0][19] = this->P2[2];
        jacobians[0][20] = 0;
        jacobians[0][21] = 0;
        jacobians[0][22] = 1;
        jacobians[0][23] = 0;

        jacobians[0][24] = 0;
        jacobians[0][25] = 0;
        jacobians[0][26] = this->P2[0];
        jacobians[0][27] = 0;
        jacobians[0][28] = 0;
        jacobians[0][29] = this->P2[1];
        jacobians[0][30] = 0;
        jacobians[0][31] = 0;
        jacobians[0][32] = this->P2[2];
        jacobians[0][33] = 0;
        jacobians[0][34] = 0;
        jacobians[0][35] = 1;
    }

    return true;
}

bool AnalyticalPointToPointInterpolated::Evaluate(double const *const *parameters, double *residuals,
                                                  double **jacobians) const {
    // modelled as P1 = T*P2
    // r = T*P2 - P1;
    // parameters in order:
    // R11 R21 R31 R12 R22 R32 R13 R23 R33 X Y Z
    Eigen::Map<const Mat34> Tmap(parameters[0], 3, 4);
    Transformation<Eigen::Map<const Mat34>> transform(Tmap);

    Transformation<> interpolated;
    interpolated.setFromExpMap(*(this->alpha) * transform.logMap());

    const Vec3 point1 = Eigen::Map<const Vec3>(this->P1, 3, 1);
    const Vec3 point2 = Eigen::Map<const Vec3>(this->P2, 3, 1);
    Vec3 err = interpolated.transform(point2) - point1;

    Eigen::Map<Vec3>(residuals, 3, 1) = err;

    if((jacobians != NULL) && (jacobians[0] != NULL)) {
        jacobians[0][0] = *(this->alpha) *this->P2[0];
        jacobians[0][1] = 0;
        jacobians[0][2] = 0;
        jacobians[0][3] = *(this->alpha) *this->P2[1];
        jacobians[0][4] = 0;
        jacobians[0][5] = 0;
        jacobians[0][6] = *(this->alpha) *this->P2[2];
        jacobians[0][7] = 0;
        jacobians[0][8] = 0;
        jacobians[0][9] = *(this->alpha) *1;
        jacobians[0][10] = 0;
        jacobians[0][11] = 0;

        jacobians[0][12] = 0;
        jacobians[0][13] = *(this->alpha) *this->P2[0];
        jacobians[0][14] = 0;
        jacobians[0][15] = 0;
        jacobians[0][16] = *(this->alpha) *this->P2[1];
        jacobians[0][17] = 0;
        jacobians[0][18] = 0;
        jacobians[0][19] = *(this->alpha) *this->P2[2];
        jacobians[0][20] = 0;
        jacobians[0][21] = 0;
        jacobians[0][22] = *(this->alpha) *1;
        jacobians[0][23] = 0;

        jacobians[0][24] = 0;
        jacobians[0][25] = 0;
        jacobians[0][26] = *(this->alpha) *this->P2[0];
        jacobians[0][27] = 0;
        jacobians[0][28] = 0;
        jacobians[0][29] = *(this->alpha) *this->P2[1];
        jacobians[0][30] = 0;
        jacobians[0][31] = 0;
        jacobians[0][32] = *(this->alpha) *this->P2[2];
        jacobians[0][33] = 0;
        jacobians[0][34] = 0;
        jacobians[0][35] = *(this->alpha) *1;
    }

    return true;
}

}