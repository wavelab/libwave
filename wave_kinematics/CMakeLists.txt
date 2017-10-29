PROJECT(wave_kinematics)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::utils
    wave::controls
    Eigen3::Eigen
    SOURCES
    src/gimbal.cpp
    src/pose.cpp
    src/quadrotor.cpp
    src/two_wheel.cpp)

# Unit tests
WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/gimbal_test.cpp
    tests/pose_test.cpp
    tests/quadrotor_test.cpp
    tests/two_wheel_test.cpp
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
