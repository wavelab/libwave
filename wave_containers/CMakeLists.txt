CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_containers)

# DEPENDENCIES
FIND_PACKAGE(Eigen3 REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(
    include
    ${EIGEN3_INCLUDE_DIR}
)

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests
    tests/containers/measurement_test.cpp
    tests/containers/landmark_measurement_test.cpp)
