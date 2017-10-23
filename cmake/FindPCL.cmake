# - Try to find PCL library
# This module wraps find_package(PCL CONFIG), then defines IMPORTED targets
#
# Once done this will define PCL, a monolithic target linking to all
# PCL_LIBRARIES and PCL_INCLUDE_DIRS for the requested components.
#
# @todo PCL's config file is messy and outdated, thus this is messy. (Maybe
# there is a better solution). Most of the effort is wrapping find_package(PCL)
# so it doesn't overwrite variables it does not own such as BOOST_LIBRARIES and
# CMAKE_CXX_FLAGS.
#
# @todo update or replace if PCL updates their package
# @author Leo Koppel <lkoppel@uwaterloo.ca>

INCLUDE(FindPackageHandleStandardArgs)

# Define a function for new scope
# This isolates all variables inappropriately set by PCLConfig.cmake
FUNCTION(_find_pcl)
    FIND_PACKAGE(PCL ${PCL_FIND_VERSION} QUIET CONFIG
        COMPONENTS ${PCL_FIND_COMPONENTS})

    ADD_LIBRARY(PCL INTERFACE IMPORTED)

    # We don't use the given PCL_LIBRARIES since it includes "debug" and "optimized"
    # keywords; instead, we specify individual modules.
    SET(imported_pcl_libraries "")

    FOREACH(component ${PCL_FIND_COMPONENTS})
        STRING(TOUPPER "${component}" component)
        LIST(APPEND imported_pcl_libraries ${PCL_${component}_LIBRARY})

        # Allow these out of the scope
        SET(PCL_${component}_FOUND ${PCL_${component}_FOUND} PARENT_SCOPE)
        SET(PCL_${component}_LIBRARY ${PCL_${component}_LIBRARY} PARENT_SCOPE)
    ENDFOREACH()

    # PCLConfig.cmake is evil for setting these
    LIST(APPEND imported_pcl_libraries
        ${BOOST_LIBRARIES}
        ${VTK_LIBRARIES})

    # This gets added directly to CMAKE_CXX_FLAGS, which we're avoiding
    LIST(APPEND PCL_DEFINITIONS ${VTK_REQUIRED_CXX_FLAGS})

    # PCL adds some entries with spaces instead of as a (semicolon-separated) CMake
    # list, which messes up later. Fix them here.
    STRING(REPLACE " " ";" pcl_definitions "${PCL_DEFINITIONS}")

    SET_TARGET_PROPERTIES(PCL PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${imported_pcl_libraries}"
        INTERFACE_COMPILE_OPTIONS "${pcl_definitions}")

    # Export these variables to the parent scope
    SET(PCL_FOUND ${PCL_FOUND} PARENT_SCOPE)
    SET(PCL_DIR ${PCL_DIR} PARENT_SCOPE)
    SET(PCL_CONFIG ${PCL_CONFIG} PARENT_SCOPE)
    SET(PCL_CONSIDERED_CONFIGS ${PCL_CONSIDERED_CONFIGS} PARENT_SCOPE)
    SET(PCL_CONSIDERED_VERSIONS ${PCL_CONSIDERED_VERSIONS} PARENT_SCOPE)

    # Note we only use the subset of libraries for the requested components
    SET(PCL_LIBRARIES ${imported_pcl_libraries} PARENT_SCOPE)
    SET(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIRS} PARENT_SCOPE)
    SET(PCL_LIBRARY_DIRS ${PCL_LIBRARY_DIRS} PARENT_SCOPE)
    SET(PCL_DEFINITIONS ${PCL_DEFINITIONS} PARENT_SCOPE)
    SET(PCL_VERSION ${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR} PARENT_SCOPE)
ENDFUNCTION(_find_pcl)
_find_pcl()

# Handle the QUIET and REQUIRED options
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PCL CONFIG_MODE)
