CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT(wave_controls)

# SETTINGS
INCLUDE_DIRECTORIES(/usr/include/eigen3)

# CONTROLS
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
ADD_LIBRARY(
    wave_controls
    STATIC
    src/pid.cpp
)
SET(
    WAVE_CONTROL_TEST_DEPS
    wave_controls
    gtest
    gtest_main
    pthread
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
ADD_EXECUTABLE(
    ${PROJECT_NAME}_tests
    tests/pid_test.cpp
)

TARGET_LINK_LIBRARIES(wave_controls_tests ${WAVE_CONTROL_TEST_DEPS})
