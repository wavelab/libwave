PROJECT(wave_kinematics)

# INCLUDES
INCLUDE_DIRECTORIES(include)

#LIBRARY
ADD_LIBRARY(
    wave_kinematics
    STATIC
    src/gimbal.cpp
    src/pose.cpp
    src/quadrotor.cpp
    src/two_wheel.cpp
)

TARGET_LINK_LIBRARIES(wave_kinematics Eigen3::Eigen)

# UNIT TESTS
WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/gimbal_test.cpp
    tests/pose_test.cpp
    tests/quadrotor_test.cpp
    tests/two_wheel_test.cpp
)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_controls
    wave_utils
)
