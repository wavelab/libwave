CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT(wave_controls)

# DEPENDENCIES
LINK_DIRECTORIES(build/deps/googletest/googletest)

# SETTINGS
INCLUDE_DIRECTORIES(/usr/include/eigen3)

# CONTROLS
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
ADD_LIBRARY(
    wave_controls
    STATIC
    src/pid.cpp
)

# TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
SET(
    TEST_DEPS
    wave_controls
    gtest
    pthread
)
ADD_EXECUTABLE(
    test_runner
    tests/control_test.cpp
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(test_runner ${TEST_DEPS})
