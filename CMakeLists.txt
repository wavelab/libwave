CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave)

# COMPILER SETTINGS
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++11)
ENDIF()

# CMAKE SETTINGS
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# INCLUDES
INCLUDE_DIRECTORIES(
  wave_controls/include
  wave_geometry/include
  wave_kinematics/include
  wave_matching/include
  wave_optimization/include
  wave_utils/include
)

# gtest
ADD_SUBDIRECTORY(deps/googletest)
INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include/ ${gtest_SOURCE_DIR})

# libwave_controls.a
ADD_SUBDIRECTORY(wave_controls)
ADD_DEPENDENCIES(wave_controls wave_utils)

# libwave_geometry.a
ADD_SUBDIRECTORY(wave_geometry)
ADD_DEPENDENCIES(wave_geometry wave_utils)

# libwave_kinematics.a
ADD_SUBDIRECTORY(wave_kinematics)
ADD_DEPENDENCIES(wave_kinematics wave_controls wave_utils)

# libwave_matching.a
ADD_SUBDIRECTORY(wave_matching)
ADD_DEPENDENCIES(wave_matching wave_utils)

# libwave_optimization.a
ADD_SUBDIRECTORY(wave_optimization)
ADD_DEPENDENCIES(wave_optimization wave_utils)

# libwave_utils.a
ADD_SUBDIRECTORY(wave_utils)

# wave_containers: header-only library
ADD_SUBDIRECTORY(wave_containers)

# libwave.a
ADD_CUSTOM_TARGET(
    libwave ALL
    COMMAND
        ${CMAKE_AR} rc libwave.a
            ${CMAKE_BINARY_DIR}/wave_controls/libwave_controls.a
	    ${CMAKE_BINARY_DIR}/wave_geometry/libwave_geometry.a
            ${CMAKE_BINARY_DIR}/wave_kinematics/libwave_kinematics.a
            ${CMAKE_BINARY_DIR}/wave_matching/libwave_matching.a
            ${CMAKE_BINARY_DIR}/wave_optimization/libwave_optimization.a
            ${CMAKE_BINARY_DIR}/wave_utils/libwave_utils.a
    DEPENDS
        wave_controls
	wave_geometry
        wave_kinematics
        wave_matching
        wave_optimization
        wave_utils
)
