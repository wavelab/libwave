CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_containers)

# DEPENDENCIES

# INCLUDES
INCLUDE_DIRECTORIES(include)

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests
    tests/measurement_test.cpp
    tests/landmark_measurement_test.cpp)
