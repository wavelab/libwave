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
    src/file.cpp
    src/math.cpp
    src/time.cpp
)

# UNIT TESTS
WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/utils/config_test.cpp
    tests/utils/data_test.cpp
    tests/utils/file_test.cpp
    tests/utils/math_test.cpp
    tests/utils/time_test.cpp
    tests/utils/template_helpers_test.cpp
)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    yaml-cpp
    ${OpenCV_LIBS}
)
# COPY TEST DATA
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
