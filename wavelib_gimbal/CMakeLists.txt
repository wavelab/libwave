CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_gimbal)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


# PACKAGES
FIND_PACKAGE(GTest REQUIRED)


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include/
    /usr/include/eigen3
    /usr/local/include
    include
)


# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
)

# LIBRARY
ADD_LIBRARY(
    slam_gimbal
    STATIC
    src/sbgc.cpp
)
SET(
    DEPS
    slam_gimbal
    slam_utils
    ${GTEST_LIBRARIES}
    pthread
)


# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(sbgc_test tests/sbgc_test.cpp)
TARGET_LINK_LIBRARIES(sbgc_test ${DEPS})
