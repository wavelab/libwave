CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_matching)

SET(CMAKE_BUILD_TYPE RELEASE)

# PACKAGES
FIND_PACKAGE(PCL 1.8 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

# DEFINITIONS
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# INCLUDES
INCLUDE_DIRECTORIES(SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS})

# LIBRARY DIRS
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

# LIBRARY
SET(
    WAVE_MATCHING_DEPS
    yaml-cpp
    wave_utils
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
ADD_LIBRARY(
    wave_matching
    STATIC
    src/gicp.cpp
    src/icp.cpp
    src/ndt.cpp
    src/pointcloud_display.cpp
)
TARGET_LINK_LIBRARIES(wave_matching ${WAVE_MATCHING_DEPS})

# UNIT TESTS
WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/icp_tests.cpp
    tests/ndt_tests.cpp
    tests/gicp_tests.cpp
    tests/multi_matcher_tests.cpp
)

WAVE_ADD_TEST(
    ${PROJECT_NAME}_viz_tests
    tests/pointcloud_display_tests.cpp
    DISABLED  # probably requires display to work properly
)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_utils
)

TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_viz_tests
    ${PROJECT_NAME}
    wave_utils
)
# Copy the test data
file(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
