#include "wave/wave_test.hpp"
#include "wave/geometry_og/transformation.hpp"
#include "wave/optimization/ceres/point_to_point_residual.hpp"
#include "wave/optimization/ceres/local_params/SE3Parameterization.hpp"
#include "wave/optimization/ceres/local_params/InterpolatedSE3.hpp"

namespace wave {

TEST(SE3Param, MinimalPointSet) {
    Vec6 transformation_twist_parameters;
    transformation_twist_parameters << 0.068924613882066, 0.213225926957886, 0.288748939228676, 0.965590777183138,
      1.960945901104432, 3.037052911306709;
    Transformation<> exact_transform;
    exact_transform.setFromExpMap(transformation_twist_parameters);

    const int size = 3;
    Vec3 points[3];
    Vec3 points_transformed[3];
    points[0] = Vec3(0, 0, 0);
    points[1] = Vec3(40, 0, 0);
    points[2] = Vec3(40, 10, 0);

    double transform[12] = {0};
    transform[0] = 1;
    transform[4] = 1;
    transform[8] = 1;

    ceres::Problem problem;
    for (int i = 0; i < size; i++) {
        points_transformed[i] = exact_transform.transform(points[i]);
        ceres::CostFunction *cost_function = new AnalyticalPointToPoint(points_transformed[i].data(), points[i].data());
        problem.AddResidualBlock(cost_function, NULL, transform);
    }
    ceres::LocalParameterization *se3 = new SE3Parameterization();
    problem.SetParameterization(transform, se3);

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Transformation<> solved_transform;
    Mat4 sol_matrix;

    sol_matrix << transform[0], transform[3], transform[6], transform[9],  //
      transform[1], transform[4], transform[7], transform[10],             //
      transform[2], transform[5], transform[8], transform[11],             //
      0, 0, 0, 1;                                                          //

    Mat4 errmat = sol_matrix - exact_transform.getMatrix();
    ASSERT_LE(errmat.norm(), 1e-5);
}

TEST(SE3Param, MinimalSetAlpha3) {
    Vec6 transformation_twist_parameters;
    double alpha = 0.3;
//    transformation_twist_parameters << 0.068924613882066, 0.213225926957886, 0.288748939228676, 0.965590777183138,
//            1.960945901104432, 3.037052911306709;
    transformation_twist_parameters << 0, 0, 0.0278, 3.6,
            0, 0.1;
    Transformation<> exact_transform, partial;
    exact_transform.setFromExpMap(transformation_twist_parameters);
    partial.setFromExpMap(alpha * transformation_twist_parameters);

    const int size = 3;
    Vec3 points[3];
    Vec3 points_transformed[3];
    points[0] = Vec3(0, 0, 0);
    points[1] = Vec3(40, 0, 0);
    points[2] = Vec3(40, 10, 0);

    double transform[12] = {0};
    transform[0] = 1;
    transform[4] = 1;
    transform[8] = 1;

    ceres::Problem problem;
    for (int i = 0; i < size; i++) {
        points_transformed[i] = partial.transform(points[i]);
        ceres::CostFunction *cost_function = new AnalyticalPointToPointInterpolated(points_transformed[i].data(), points[i].data(), &alpha);
        problem.AddResidualBlock(cost_function, NULL, transform);
    }

    ceres::LocalParameterization *se3 = new SE3Parameterization;
    problem.SetParameterization(transform, se3);

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Transformation<> solved_transform;
    Mat4 sol_matrix;

    sol_matrix << transform[0], transform[3], transform[6], transform[9],  //
            transform[1], transform[4], transform[7], transform[10],             //
            transform[2], transform[5], transform[8], transform[11],             //
            0, 0, 0, 1;                                                          //

    Mat4 errmat = sol_matrix - exact_transform.getMatrix();
    ASSERT_LE(errmat.norm(), 1e-5);
}

TEST(InterpolateParam, MinimalPointSetAlpha1) {
    Vec6 transformation_twist_parameters;
    transformation_twist_parameters << 0.068924613882066, 0.213225926957886, 0.288748939228676, 0.965590777183138,
            1.960945901104432, 3.037052911306709;
    Transformation<> exact_transform;
    exact_transform.setFromExpMap(transformation_twist_parameters);

    const int size = 3;
    Vec3 points[3];
    Vec3 points_transformed[3];
    points[0] = Vec3(0, 0, 0);
    points[1] = Vec3(40, 0, 0);
    points[2] = Vec3(40, 10, 0);

    double transform[12] = {0};
    transform[0] = 1;
    transform[4] = 1;
    transform[8] = 1;

    ceres::Problem problem;
    for (int i = 0; i < size; i++) {
        points_transformed[i] = exact_transform.transform(points[i]);
        ceres::CostFunction *cost_function = new AnalyticalPointToPoint(points_transformed[i].data(), points[i].data());
        problem.AddResidualBlock(cost_function, NULL, transform);
    }
    double alpha = 1;
    ceres::LocalParameterization *se3 = new InterpolatedSE3(alpha);
    problem.SetParameterization(transform, se3);

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Transformation<> solved_transform;
    Mat4 sol_matrix;

    sol_matrix << transform[0], transform[3], transform[6], transform[9],  //
            transform[1], transform[4], transform[7], transform[10],             //
            transform[2], transform[5], transform[8], transform[11],             //
            0, 0, 0, 1;                                                          //

    Mat4 errmat = sol_matrix - exact_transform.getMatrix();
    ASSERT_LE(errmat.norm(), 1e-5);
}

/**
 * Solution accuracy is worse with large twist values.
 */
TEST(InterpolateParam, MinimalSetAlpha3) {
    Vec6 transformation_twist_parameters;
    double alpha = 0.3;
    transformation_twist_parameters << 0.068924613882066, 0.213225926957886, 0.288748939228676, 0.965590777183138,
            1.960945901104432, 3.037052911306709;
    Transformation<> exact_transform, partial;
    exact_transform.setFromExpMap(transformation_twist_parameters);
    partial.setFromExpMap(alpha * transformation_twist_parameters);

    const int size = 3;
    Vec3 points[3];
    Vec3 points_transformed[3];
    points[0] = Vec3(0, 0, 0);
    points[1] = Vec3(40, 0, 0);
    points[2] = Vec3(40, 10, 0);

    double transform[12] = {0};
    transform[0] = 1;
    transform[4] = 1;
    transform[8] = 1;

    ceres::Problem problem;
    for (int i = 0; i < size; i++) {
        points_transformed[i] = partial.transform(points[i]);
        ceres::CostFunction *cost_function = new AnalyticalPointToPointInterpolated(points_transformed[i].data(), points[i].data(), &alpha);
        problem.AddResidualBlock(cost_function, NULL, transform);
    }

    ceres::LocalParameterization *se3 = new InterpolatedSE3(alpha);
    problem.SetParameterization(transform, se3);

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    Transformation<> solved_transform;
    Mat4 sol_matrix;

    sol_matrix << transform[0], transform[3], transform[6], transform[9],  //
            transform[1], transform[4], transform[7], transform[10],             //
            transform[2], transform[5], transform[8], transform[11],             //
            0, 0, 0, 1;                                                          //

    Mat4 errmat = sol_matrix - exact_transform.getMatrix();
    ASSERT_LE(errmat.norm(), 1e-3);
}

}