PROJECT(wave_matching)

SET(CMAKE_BUILD_TYPE RELEASE)

# PACKAGES

# Define a monolithic imported target for PCL
FIND_PACKAGE(PCL 1.8 REQUIRED
    common filters registration kdtree search io visualization)
ADD_LIBRARY(PCL INTERFACE IMPORTED)

# @todo PCL's config file is messy and outdated (overwriting BOOST_LIBRARIES,
# for example), thus this is messy. (Maybe there is a better solution).
# We don't use the given PCL_LIBRARIES since it includes "debug" and "optimized"
# keywords; instead, we specify individual modules. Update if more are needed.
SET(PCL_LIBRARIES ${PCL_COMMON_LIBRARY}
    ${PCL_FILTERS_LIBRARY}
    ${PCL_REGISTRATION_LIBRARY}
    ${PCL_KDTREE_LIBRARY}
    ${PCL_SEARCH_LIBRARY}
    ${PCL_IO_LIBRARY}
    ${PCL_VISUALIZATION_LIBRARY}
    ${BOOST_LIBRARIES}
    ${VTK_LIBRARIES})

# PCL adds some entries with spaces instead of as a (semicolon-separated) CMake
# list, which messes up later. Fix them here.
string(REPLACE " " ";" pcl_definitions "${PCL_DEFINITIONS}")

SET_TARGET_PROPERTIES(PCL PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${PCL_LIBRARIES}"
    INTERFACE_COMPILE_OPTIONS "${pcl_definitions}")

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
