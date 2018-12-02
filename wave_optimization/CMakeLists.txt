PROJECT(wave_optimization)
# For now, just some examples of using ceres @todo

WAVE_ADD_MODULE(${PROJECT_NAME} DEPENDS
    wave::utils
    wave::kinematics
    wave::geometry
    #wave::vision
    Eigen3::Eigen
    ceres
    SOURCES
    #src/ceres/ba.cpp
    src/ceres/ceres_examples.cpp
    src/ceres/local_params/SE3Parameterization.cpp
    src/ceres/local_params/null_SE3_parameterization.cpp
    src/ceres/local_params/null_SE3_parameterization_remap.cpp
    src/ceres/local_params/null_SE3_translation_only.cpp
    src/ceres/local_params/spherical_parameterization.cpp
    src/ceres/local_params/line_parameterization.cpp
    src/ceres/local_params/plane_parameterization.cpp
    src/ceres/odom_gp/point_to_line_gp.cpp
    src/ceres/odom_gp/point_to_plane_gp.cpp
    src/ceres/odom_gp/constant_velocity.cpp
    src/ceres/odom_gp_twist/point_to_line_gp.cpp
    src/ceres/odom_gp_twist/point_to_plane_gp.cpp
    src/ceres/odom_gp_twist/constant_velocity.cpp
    src/ceres/odom_gp_reduced/point_to_line_gp.cpp
    src/ceres/odom_gp_reduced/point_to_plane_gp.cpp
    src/ceres/odom_gp_reduced/constant_velocity.cpp
    src/ceres/local_params/InterpolatedSE3.cpp
    src/ceres/point_to_point_residual.cpp
    src/ceres/loss_function/bisquare_loss.cpp
    src/ceres/loss_function/quartic_loss.cpp
    src/ceres/odom_linear/point_to_line_interpolated_transform.cpp
    src/ceres/odom_linear/point_to_plane_interpolated_transform.cpp
    src/ceres/tiny/robust_line_fit.cpp)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
                 #tests/ceres/ba_test.cpp
                 tests/ceres/ceres_examples_test.cpp
                 tests/ceres/SE3_local_parameter_test.cpp
                 tests/ceres/solution_remapping_test.cpp
                 tests/ceres/spherical_param_test.cpp)

    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})

    WAVE_ADD_TEST(odometry_residual_tests
            tests/ceres/point_to_line_test.cpp
            tests/ceres/point_to_plane_test.cpp)

    TARGET_LINK_LIBRARIES(odometry_residual_tests
            ${PROJECT_NAME})

    WAVE_ADD_TEST(geometry_residual_tests
            tests/ceres/transform_prior_test.cpp
            tests/ceres/trajectory_prior.cpp
            tests/ceres/constant_velocity_test.cpp
            tests/ceres/ceres_local_jacobian_test.cpp)

    TARGET_LINK_LIBRARIES(geometry_residual_tests
            ${PROJECT_NAME})

    WAVE_ADD_TEST(manual_jacobian_tests
            tests/ceres/gp_jacobian_test.cpp)

    TARGET_LINK_LIBRARIES(manual_jacobian_tests
            ${PROJECT_NAME})

    WAVE_ADD_TEST(tiny_tests
            tests/ceres/tiny/robust_line_fit_test.cpp)
    TARGET_LINK_LIBRARIES(tiny_tests
            ${PROJECT_NAME}
            wave::matching)

    # COPY TEST DATA
    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
ENDIF(BUILD_TESTING)
