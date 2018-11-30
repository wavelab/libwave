PROJECT(wave_kinematics)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::utils
    wave::controls
    wave::geometry_og
    Eigen3::Eigen
    SOURCES
    src/gimbal.cpp
    src/pose.cpp
    src/quadrotor.cpp
    src/two_wheel.cpp
    src/constant_velocity_gp_prior.cpp)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
        tests/gimbal_test.cpp
        tests/pose_test.cpp
        tests/quadrotor_test.cpp
        tests/two_wheel_test.cpp)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
ENDIF(BUILD_TESTING)
