CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_kinematics)

IF (CMAKE_COMPILER_IS_GNUCXX)
  ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# INCLUDES
INCLUDE_DIRECTORIES(
  /usr/include
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
# ADD_LIBRARY(
#   wave_kinematics
#   STATIC
#   src/
# )
SET(
  DEPS
  wave_utils
  gtest
  pthread
)


# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)

# ADD_EXECUTABLE(data_test tests/data_test.cpp)
# TARGET_LINK_LIBRARIES(data_test ${DEPS})
