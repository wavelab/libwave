PROJECT(wave_containers)

# Define a header-only library
WAVE_ADD_LIBRARY(${PROJECT_NAME})
WAVE_INCLUDE_DIRECTORIES(${PROJECT_NAME} INTERFACE include)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} INTERFACE
    Eigen3::Eigen
    Boost::boost)

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests
    tests/measurement_test.cpp
    tests/landmark_measurement_test.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
