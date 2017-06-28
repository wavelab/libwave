CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_odometry)

# PACKAGES
FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(
    include
    /usr/include/eigen3
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

# LIBRARY
ADD_LIBRARY(
    wave_odometry
    STATIC
    src/LaserOdom.cpp
)

LINK_DIRECTORIES(
    ${PCL_LIBRARY_DIRS})

# UNIT TESTS
#WAVE_ADD_TEST(${PROJECT_NAME}_tests
#        tests/ceres/ceres_examples_test.cpp)
TARGET_LINK_LIBRARIES(
    #${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_utils
    ${CERES_LIBRARIES}
    ${PCL_LIBRARIES}
)
