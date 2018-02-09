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
    src/ceres/ba.cpp
    src/ceres/ceres_examples.cpp
    src/ceres/SE3Parameterization.cpp
    src/ceres/null_SE3_parameterization.cpp
    src/ceres/null_SE3_parameterization_remap.cpp
    src/ceres/point_to_line_gp.cpp
    src/ceres/point_to_plane_gp.cpp
    src/ceres/InterpolatedSE3.cpp
    src/ceres/point_to_point_residual.cpp
    src/ceres/bisquare_loss.cpp
    src/ceres/quartic_loss.cpp
    src/ceres/point_to_line_interpolated_transform.cpp
    src/ceres/point_to_plane_interpolated_transform.cpp
    src/ceres/constant_velocity.cpp)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
                 #tests/ceres/ba_test.cpp
                 tests/ceres/ceres_examples_test.cpp
                 tests/ceres/SE3_local_parameter_test.cpp
                 tests/ceres/solution_remapping_test.cpp)

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

    # COPY TEST DATA
    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
ENDIF(BUILD_TESTING)
