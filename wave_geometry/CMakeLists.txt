CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_geometry)

# EXTERNAL PACKAGES
FIND_PACKAGE(Kindr)

# INCLUDES
INCLUDE_DIRECTORIES(/usr/include/eigen3)

#LIBRARY
ADD_LIBRARY(
    wave_geometry
    STATIC
    src/rotation.cpp
)
SET(
    WAVE_GEOMETRY_TEST_DEPS
    wave_geometry
    wave_utils
    gtest
    gtest_main
    pthread
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
ADD_EXECUTABLE(
    ${PROJECT_NAME}_tests
    tests/rotation_test.cpp
)
TARGET_LINK_LIBRARIES(wave_geometry_tests ${WAVE_GEOMETRY_TEST_DEPS})
