CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave)

# COMPILER SETTINGS
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++11)
ENDIF()

# INCLUDES
INCLUDE_DIRECTORIES(
  wave_estimation/include
  wave_kinematics/include
  wave_optimization/include
  wave_utils/include
  wave_matching/include
  wave_vision/include
)

# gtest
ADD_SUBDIRECTORY(deps/googletest)
INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include/)

# libwave_optimization.a
ADD_SUBDIRECTORY(wave_optimization)
ADD_DEPENDENCIES(wave_optimization wave_utils)

# libwave_utils.a
ADD_SUBDIRECTORY(wave_utils)

# libwave_vision.a
ADD_SUBDIRECTORY(wave_vision)
ADD_DEPENDENCIES(wave_vision wave_utils)

# libwave_matching.a
ADD_SUBDIRECTORY(wave_matching)
ADD_DEPENDENCIES(wave_matching wave_utils)

# libwave.a
ADD_CUSTOM_TARGET(
    libwave ALL
    COMMAND
        ${CMAKE_AR} rc libwave.a
            ${CMAKE_BINARY_DIR}/wave_optimization/libwave_optimization.a
            ${CMAKE_BINARY_DIR}/wave_utils/libwave_utils.a
            ${CMAKE_BINARY_DIR}/wave_vision/libwave_vision.a
            ${CMAKE_BINARY_DIR}/wave_matching/libwave_matching.a
    DEPENDS
        wave_optimization
        wave_utils
        wave_vision
        wave_matching
)
