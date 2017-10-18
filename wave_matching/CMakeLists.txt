PROJECT(wave_matching)

SET(CMAKE_BUILD_TYPE RELEASE)

# PACKAGES
FIND_PACKAGE(PCL 1.8 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

ADD_LIBRARY(${PROJECT_NAME} STATIC
    src/gicp.cpp
    src/icp.cpp
    src/ndt.cpp
    src/pointcloud_display.cpp)

# Use these headers when building, and make clients use them
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

# Use these compile definitions when building our library and clients
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC ${PCL_DEFINITIONS})

# Pass these external library to the linker (@todo remove if possible)
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    wave_utils
    Eigen3::Eigen
    yaml-cpp
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES})

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
