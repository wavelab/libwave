CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT(wave_quadrotor)

# DEPENDENCIES
FIND_PACKAGE(OpenCV REQUIRED)
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
    wave_quadrotor
    STATIC
    src/control.cpp
    src/quadrotor.cpp
)
ADD_DEPENDENCIES(
    wave_utils
)
INSTALL(
    TARGETS wave_quadrotor
    LIBRARY ARCHIVE DESTINATION ${LIB_DEST}
)
INSTALL(
    TARGETS wave_quadrotor
    LIBRARY ARCHIVE DESTINATION ${ROS_LIB_DEST}
)

# TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
SET(
    TEST_DEPS
    wave_quadrotor
    wave_utils
    yaml-cpp
    gtest
    pthread
    ${OpenCV_LIBS}
)
ADD_EXECUTABLE(
    test_runner
    tests/control_test.cpp
    tests/quadrotor_test.cpp
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(test_runner ${TEST_DEPS})
