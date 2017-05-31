CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_matching)

# PACKAGES
FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

# DEFINITIONS
ADD_DEFINITIONS(${EIGEN_DEFINITIONS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# INCLUDES
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

# LIBRARY DIRS
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
LINK_DIRECTORIES(${EIGEN_LIBRARY_DIRS})
LINK_DIRECTORIES(/usr/lib/x86_64-linux-gnu/)

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
    tests/pointcloud_display_tests.cpp
    DISABLED  # Don't run this test for now
)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_utils
)
# Copy the test data
file(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
