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

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests tests/rotation_test.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_utils)
