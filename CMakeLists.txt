CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
PROJECT(wave)

# Package version, used when other projects FIND_PACKAGE(Wave <version>)
SET(WAVE_PACKAGE_VERSION 0.1.0)

# Compiler settings for all targets
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# CMake modules
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
INCLUDE(CMakePackageConfigHelpers)
INCLUDE(cmake/WaveUtils.cmake)
ENABLE_TESTING()

# Find common dependencies (used by multiple required modules)

# Require Eigen 3.2.92, also called 3.3 beta-1, since it's in xenial
FIND_PACKAGE(Eigen3 3.2.92 REQUIRED)
FIND_PACKAGE(Boost 1.54.0 REQUIRED system filesystem)
FIND_PACKAGE(PCL 1.8 REQUIRED
    common filters registration kdtree search io visualization)
FIND_PACKAGE(kindr REQUIRED)

# Where dependencies do not provide imported targets, define them
INCLUDE(cmake/ImportEigen3.cmake)
INCLUDE(cmake/ImportBoost.cmake)
INCLUDE(cmake/ImportPCL.cmake)
INCLUDE(cmake/ImportKindr.cmake)

# Add a special "wave" target including all modules
# The WAVE_ADD_LIBRARY helper in WaveUtils.cmake will add each module to this
ADD_LIBRARY(wave INTERFACE)
INSTALL(TARGETS wave EXPORT WaveTargets)

# Add each module to the project
ADD_SUBDIRECTORY(wave_benchmark)
ADD_SUBDIRECTORY(wave_controls)
ADD_SUBDIRECTORY(wave_containers)
ADD_SUBDIRECTORY(wave_geometry)
ADD_SUBDIRECTORY(wave_kinematics)
ADD_SUBDIRECTORY(wave_matching)
ADD_SUBDIRECTORY(wave_optimization)
ADD_SUBDIRECTORY(wave_utils)
ADD_SUBDIRECTORY(wave_vision)

# gtest
ADD_SUBDIRECTORY(deps/googletest EXCLUDE_FROM_ALL)
INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include/ ${gtest_SOURCE_DIR})

# Documentation
SET(WAVE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(WAVE_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
ADD_SUBDIRECTORY(docs)

# This is where .cmake files will be installed (default under share/)
SET(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_DATADIR}/wave/cmake" CACHE PATH
    "Installation directory for CMake files")

# Add an export file listing all targets
EXPORT(EXPORT WaveTargets NAMESPACE Wave:: FILE WaveTargets.cmake)
# Install the export file
INSTALL(EXPORT WaveTargets NAMESPACE Wave:: DESTINATION "${INSTALL_CMAKE_DIR}")

# Export this package so it can be found through CMake's registry without being
# installed.
EXPORT(PACKAGE Wave)

# Create the WaveConfig and WaveConfigVersion files

# Set variables used to generate the Config file
# Using these is "not recommended". For example, WAVE_INCLUDE_DIRS is not
# relocatable; WAVE_LIBRARIES gives names but not paths
SET(WAVE_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}")
GET_TARGET_PROPERTY(WAVE_LIBRARIES wave INTERFACE_LINK_LIBRARIES)

# Generate the Config file for the install tree
SET(WAVE_EXTRA_CMAKE_DIR "cmake")
CONFIGURE_PACKAGE_CONFIG_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/WaveConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/WaveConfig.cmake"
    PATH_VARS WAVE_INCLUDE_DIRS WAVE_LIBRARIES
    INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}")

# Generate the Config file for the build tree
# They differ in where our bundled Import*.cmake scripts are included from
# Here, use the relative path from the build directory to this source directory
FILE(RELATIVE_PATH path_to_wave_source
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR})
SET(WAVE_EXTRA_CMAKE_DIR "${path_to_wave_source}cmake")
CONFIGURE_PACKAGE_CONFIG_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/WaveConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/WaveConfig.cmake"
    PATH_VARS WAVE_INCLUDE_DIRS WAVE_LIBRARIES
    INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}")




# Generate the Version file
WRITE_BASIC_PACKAGE_VERSION_FILE(WaveConfigVersion.cmake
    VERSION ${WAVE_PACKAGE_VERSION}
    COMPATIBILITY SameMajorVersion)

# Install the Config and ConfigVersion files
INSTALL(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/WaveConfig.cmake"
    "${PROJECT_BINARY_DIR}/WaveConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}")

# Install our import scripts so we can use them in the Config script. That way,
# other projects using libwave will get transient dependencies automatically
INSTALL(DIRECTORY cmake/
    DESTINATION "${INSTALL_CMAKE_DIR}/cmake"
    FILES_MATCHING PATTERN "Import*.cmake")
