CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_kinematics)

# INCLUDES
INCLUDE_DIRECTORIES(/usr/include/eigen3)

#LIBRARY
ADD_LIBRARY(
    wave_kinematics
    STATIC
    src/gimbal.cpp
    src/pose.cpp
    src/quadrotor.cpp
)

# UNIT TESTS
WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/gimbal_test.cpp
    tests/pose_test.cpp
    tests/quadrotor_test.cpp
)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_controls
    wave_utils
)
