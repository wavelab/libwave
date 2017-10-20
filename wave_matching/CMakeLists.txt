PROJECT(wave_matching)

SET(CMAKE_BUILD_TYPE RELEASE)

# PACKAGES

# Pass these external library to the linker (@todo remove if possible)
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

# Declare our library
ADD_LIBRARY(${PROJECT_NAME} STATIC
    src/gicp.cpp
    src/icp.cpp
    src/ndt.cpp
    src/pointcloud_display.cpp)

# Use these headers when building, and make clients use them
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    wave_utils
    Eigen3::Eigen
    Boost::boost
    PCL)


# Set install destinations for all related files
WAVE_INSTALL(${PROJECT_NAME})

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
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_viz_tests ${PROJECT_NAME})

# Copy the test data
file(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
