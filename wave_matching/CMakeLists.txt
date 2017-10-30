PROJECT(wave_matching)

SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# PACKAGES
FIND_PACKAGE(PCL 1.8 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
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

# Declare our library
WAVE_ADD_LIBRARY(${PROJECT_NAME}
    src/gicp.cpp
    src/icp.cpp
    src/ndt.cpp
    src/ground_segmentation.cpp
    src/pointcloud_display.cpp
)
# Use these headers when building, and make clients use them
WAVE_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    wave::utils
    Eigen3::Eigen
    Boost::boost
    PCL::PCL)

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

WAVE_ADD_TEST(${PROJECT_NAME}_ground_segmentation_tests
        tests/ground_segmentation_test.cpp
        DISABLED  # Uses viewer to check results
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

TARGET_LINK_LIBRARIES(
        ${PROJECT_NAME}_ground_segmentation_tests
        ${PROJECT_NAME}
        wave_utils
)
# Copy the test data
file(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
