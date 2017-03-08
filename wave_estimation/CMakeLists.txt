CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_estimation)

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
    wave_estimation
    STATIC
    src/kf.cpp
    src/ekf.cpp
)
SET(
    TEST_DEPS
    wave_estimation
    wave_utils
    gtest
    pthread
)


# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(kf_test tests/kf_test.cpp)
TARGET_LINK_LIBRARIES(kf_test ${TEST_DEPS})

# ADD_EXECUTABLE(ekf_test tests/ekf_test.cpp)
# TARGET_LINK_LIBRARIES(ekf_test ${TEST_DEPS})
