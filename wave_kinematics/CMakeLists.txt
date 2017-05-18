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
SET(
    WAVE_KINEMATICS_TEST_DEPS
    wave_kinematics
    wave_controls
    wave_utils
    gtest
    gtest_main
    pthread
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
ADD_EXECUTABLE(
    wave_kinematics_tests
    # TESTS
    tests/gimbal_test.cpp
    tests/pose_test.cpp
    tests/quadrotor_test.cpp
)
TARGET_LINK_LIBRARIES(wave_kinematics_tests ${WAVE_KINEMATICS_TEST_DEPS})
