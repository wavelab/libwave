#include "wave/wave_test.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/optimization/ceres/point_to_plane_interpolated_transform.hpp"
#include "wave/optimization/ceres/SE3Parameterization.hpp"

namespace wave {

TEST(Residual_test, SE3pointToPlaneAnalytic) {
    const double **trans;
    trans = new const double *[1];
    trans[0] = new const double[12]{0.999613604886095,
                                    0.027796419313034,
                                    0,
                                    -0.027796419313034,
                                    0.999613604886095,
                                    0,
                                    0,
                                    0,
                                    1,
                                    3.599536313918120,
                                    0.050036777340220,
                                    0};

    double **jacobian;
    jacobian = new double *[1];
    jacobian[0] = new double[12];

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, 0};
    double ptC[3] = {4, -1, 0};
    double pt[3] = {1, 2, -4};
    double scale = 0.6;
    double residual = 0;

    SE3PointToPlane thing(pt, ptA, ptB, ptC, &scale, 1.0);
    thing.Evaluate(trans, &residual, jacobian);

    EXPECT_NEAR(residual, -4, 1e-4);

    EXPECT_NEAR(jacobian[0][0], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][1], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][2], 0.6, 1e-4);
    EXPECT_NEAR(jacobian[0][3], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][4], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][5], 1.2, 1e-4);
    EXPECT_NEAR(jacobian[0][6], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][7], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][8], -2.4, 1e-4);
    EXPECT_NEAR(jacobian[0][9], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][10], 0, 1e-4);
    EXPECT_NEAR(jacobian[0][11], 0.6, 1e-4);
}

}
