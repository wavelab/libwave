CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_viz)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)

# PACKAGES
FIND_PACKAGE(GTest REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)


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
    slam_viz
    STATIC
    src/viz.cpp
    src/vizcamera.cpp
)
SET(GL_DEPS GL GLU X11)
SET(GTEST_DEPS ${GTEST_LIBRARIES} pthread)
SET(
    DEPS
    slam_viz
    slam_utils
    yaml-cpp
    ${SDL2_LIBRARIES}
    ${GL_DEPS}
    ${GTEST_DEPS}
)


# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
# FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
# FILE(COPY scripts DESTINATION ${PROJECT_BINARY_DIR})

ADD_EXECUTABLE(viz_test tests/viz_test.cpp)
TARGET_LINK_LIBRARIES(viz_test ${DEPS})
