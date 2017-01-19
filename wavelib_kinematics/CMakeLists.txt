CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_kinematics)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()


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
    slam_kinematics
    STATIC
    src/quadrotor.cpp
    src/twowheel.cpp
)


# UNIT TESTS
# SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

# ADD_EXECUTABLE(esitmation_test tests/esitmation_test.cpp)
# TARGET_LINK_LIBRARIES(esitmation_test ${DEPS})
