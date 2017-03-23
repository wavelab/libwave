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
)

# gtest
ADD_SUBDIRECTORY(deps/googletest)
INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include/)

# libwave_estimation.a
ADD_SUBDIRECTORY(wave_estimation)
ADD_DEPENDENCIES(wave_estimation wave_utils)

# libwave_kinematics.a
ADD_SUBDIRECTORY(wave_kinematics)
ADD_DEPENDENCIES(wave_kinematics wave_utils)

# # libwave_optimization.a
# ADD_SUBDIRECTORY(wave_optimization)
# ADD_DEPENDENCIES(wave_optimization wave_utils)

# libwave_utils.a
ADD_SUBDIRECTORY(wave_utils)

# libwave.a
ADD_CUSTOM_TARGET(
  libwave ALL
  COMMAND
    ${CMAKE_AR} rc libwave.a
      ${CMAKE_BINARY_DIR}/wave_estimation/libwave_estimation.a
      ${CMAKE_BINARY_DIR}/wave_kinematics/libwave_kinematics.a
      # ${CMAKE_BINARY_DIR}/wave_estimation/libwave_optimization.a
      ${CMAKE_BINARY_DIR}/wave_utils/libwave_utils.a
  DEPENDS
    wave_estimation
    wave_kinematics
    wave_utils
)
