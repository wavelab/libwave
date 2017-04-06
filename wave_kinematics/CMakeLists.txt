CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT(wave_kinematics)

# DEPENDENCIES
LINK_DIRECTORIES(build/deps/googletest/googletest)

# SETTINGS
INCLUDE_DIRECTORIES(
    /usr/local/include
    /usr/include/eigen3
    ${gtest_SOURCE_DIR}/include
    ${gtest_SOURCE_DIR}
)

# KINEMATICS
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
ADD_LIBRARY(
    wave_kinematics
    STATIC
    src/quadrotor.cpp
)

# TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
SET(
    TEST_DEPS
    wave_kinematics
    wave_controls
    wave_utils
#    yaml-cpp
    gtest
#    pthread
#    ${OpenCV_LIBS}
)
ADD_EXECUTABLE(
    wave_kinematics_tests
    tests/quadrotor_test.cpp
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(wave_kinematics_tests ${TEST_DEPS})
