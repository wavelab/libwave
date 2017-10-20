CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
PROJECT(wave)

# COMPILER SETTINGS
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# CMAKE SETTINGS
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
INCLUDE(cmake/WaveUtils.cmake)
ENABLE_TESTING()

# Find common dependencies (used by multiple required modules)

# Require Eigen 3.2.92, also called 3.3 beta-1, since it's in xenial
FIND_PACKAGE(Eigen3 3.2.92 REQUIRED)

FIND_PACKAGE(Boost 1.54.0 REQUIRED system filesystem)

# Use our FindPCL module to import a monolithic target for PCL
FIND_PACKAGE(PCL 1.8 REQUIRED
    common filters registration kdtree search io visualization)

# The sub-libraries, each of which is in a subdirectory of the same name
SET(LIBWAVE_MODULES
    wave_controls
    wave_containers
    wave_geometry
    wave_kinematics
    wave_matching
    wave_optimization
    wave_vision
    wave_utils
    wave_benchmark)

FOREACH(dir ${LIBWAVE_MODULES})
    ADD_SUBDIRECTORY(${dir})
ENDFOREACH()

# gtest
ADD_SUBDIRECTORY(deps/googletest EXCLUDE_FROM_ALL)
INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include/ ${gtest_SOURCE_DIR})

# Documentation
SET(WAVE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(WAVE_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
ADD_SUBDIRECTORY(docs)
