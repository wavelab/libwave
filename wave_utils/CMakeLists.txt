CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_utils)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# DEPENDENCIES
FIND_PACKAGE(OpenCV 3.0 REQUIRED)

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
ADD_LIBRARY(
    wave_utils
    STATIC
    src/config.cpp
    src/data.cpp
    src/filesystem.cpp
    src/math.cpp
    src/time.cpp
)
SET(
    TEST_DEPS
    wave_utils
    yaml-cpp
    gtest
    pthread
    ${OpenCV_LIBS}
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(
    # TARGET
    wave_utils_tests
    # TESTS
    tests/utils/config_test.cpp
    tests/utils/data_test.cpp
    tests/utils/filesystem_test.cpp
    tests/utils/math_test.cpp
    tests/utils/time_test.cpp
    # TEST RUNNER
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(wave_utils_tests ${TEST_DEPS})
