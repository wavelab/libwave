CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_optimization)

# PACKAGES
FIND_PACKAGE(Ceres REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

# LIBRARY
ADD_LIBRARY(
    wave_optimization
    STATIC
    src/ceres/ceres_examples.cpp
)
SET(
    TEST_DEPS
    wave_optimization
    wave_utils
    ${CERES_LIBRARIES}
    gtest
    pthread
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
ADD_EXECUTABLE(
    # TARGET
    wave_optimization_tests
    # TESTS
    tests/ceres/ceres_examples_test.cpp
    tests/factor_graph/factor_graph_test.cpp
    tests/factor_graph/factor_test.cpp
    tests/factor_graph/variable_test.cpp
    # TEST RUNNER
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(wave_optimization_tests ${TEST_DEPS})
