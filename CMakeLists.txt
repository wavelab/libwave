CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
PROJECT(wave)

# Package version, used when other projects FIND_PACKAGE(Wave <version>)
SET(WAVE_PACKAGE_VERSION 0.1.0)

# COMPILER SETTINGS
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# CMAKE SETTINGS
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
INCLUDE(cmake/WaveUtils.cmake)
ENABLE_TESTING()

# Find common dependencies (used by multiple required modules)

# Require Eigen 3.2.92, also called 3.3 beta-1, since it's in xenial
FIND_PACKAGE(Eigen3 3.2.92 REQUIRED)

FIND_PACKAGE(Boost 1.54.0 REQUIRED system filesystem)

# Use our FindPCL module to import a monolithic target for PCL
FIND_PACKAGE(PCL 1.8 REQUIRED
    common filters registration kdtree search io visualization)

# The modules, each of which is in a subdirectory of the same name
SET(LIBWAVE_MODULES
    wave_controls
    wave_containers
    wave_geometry
    wave_kinematics
    wave_matching
    wave_optimization
    wave_vision
    wave_utils
    wave_benchmark)

FOREACH(dir ${LIBWAVE_MODULES})
    ADD_SUBDIRECTORY(${dir})
ENDFOREACH()

# gtest
ADD_SUBDIRECTORY(deps/googletest EXCLUDE_FROM_ALL)
INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include/ ${gtest_SOURCE_DIR})

# Documentation
SET(WAVE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(WAVE_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
ADD_SUBDIRECTORY(docs)

# This is where .cmake files will be installed (default under share/)
set(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_DATADIR}/wave/cmake" CACHE PATH
    "Installation directory for CMake files")

# Add an export file listing all targets
EXPORT(EXPORT WaveTargets NAMESPACE Wave:: FILE WaveTargets.cmake)
# Install the export file
INSTALL(EXPORT WaveTargets NAMESPACE Wave:: DESTINATION "$INSTALL_CMAKE_DIR")

# Export this package so it can be found through CMake's registry without being
# installed.
EXPORT(PACKAGE Wave)

# Create the WaveConfig and WaveConfigVersion files
INCLUDE(CMakePackageConfigHelpers)
# Set variables used in the Config file
SET(WAVE_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}")
# Generate the Config file
CONFIGURE_PACKAGE_CONFIG_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/WaveConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/WaveConfig.cmake"
    PATH_VARS WAVE_INCLUDE_DIRS
    INSTALL_DESTINATION "INSTALL_CMAKE_DIR")
# Generate the Version file
WRITE_BASIC_PACKAGE_VERSION_FILE(WaveConfigVersion.cmake
    VERSION ${WAVE_PACKAGE_VERSION}
    COMPATIBILITY SameMajorVersion)


# Install the WaveConfig and WaveConfigVersion files
install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/WaveConfig.cmake"
    "${PROJECT_BINARY_DIR}/WaveConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}")
