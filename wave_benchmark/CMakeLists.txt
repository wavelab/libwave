CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_benchmark)

# DEPENDENCIES
FIND_PACKAGE(Eigen3 REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(
    include
    /usr/include/eigen3
)

ADD_LIBRARY(${PROJECT_NAME} src/trajectorycompare.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} wave_utils wave_geometry)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(
    # TARGET
    ${PROJECT_NAME}_tests
    # TESTS
    tests/benchmark/benchmark_tests.cpp
    # TEST RUNNER
    tests/test_runner.cpp
)

SET(TEST_DEPS gtest ${PROJECT_NAME})

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${TEST_DEPS})
