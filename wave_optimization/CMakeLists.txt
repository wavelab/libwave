PROJECT(wave_optimization)
# For now, just some examples of using ceres @todo

WAVE_ADD_MODULE(${PROJECT_NAME} DEPENDS
    wave::utils
    wave::kinematics
    wave::vision
    Eigen3::Eigen
    ceres
    SOURCES
    src/ceres/ba.cpp
    src/ceres/ceres_examples.cpp
    src/ceres/SE3Parameterization.cpp
    src/ceres/InterpolatedSE3.cpp
    src/ceres/point_to_point_residual.cpp)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
                 tests/ceres/ba_test.cpp
                 tests/ceres/ceres_examples_test.cpp
                 tests/ceres/SE3_local_parameter_test.cpp)

    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})

    # COPY TEST DATA
    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
ENDIF(BUILD_TESTING)
