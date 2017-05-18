CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_utils)

# DEPENDENCIES
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

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
    WAVE_UTILS_TEST_DEPS
    wave_utils
    yaml-cpp
    gtest
    gtest_main
    pthread
    ${OpenCV_LIBS}
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
ADD_EXECUTABLE(
    ${PROJECT_NAME}_tests
    tests/utils/config_test.cpp
    tests/utils/data_test.cpp
    tests/utils/filesystem_test.cpp
    tests/utils/math_test.cpp
    tests/utils/time_test.cpp
)
TARGET_LINK_LIBRARIES(wave_utils_tests ${WAVE_UTILS_TEST_DEPS})
