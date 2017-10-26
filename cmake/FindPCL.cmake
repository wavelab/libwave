# - Try to find PCL library
# This module wraps find_package(PCL CONFIG) to make these small changes:
#
# * It doesn't overwrite variables it does not own such as BOOST_LIBRARIES and
#   CMAKE_CXX_FLAGS.
# * The list PCL_LIBRARIES includes all transitive dependencies such as
#   BOOST_LIBRARIES and VTK_LIBRARIES
# * The list PCL_DEFINITIONS is correctly formatted and contains all flags which
#   would otherwise be added directly to CMAKE_CXX_FLAGS
#
# @author Leo Koppel <lkoppel@uwaterloo.ca>

INCLUDE(FindPackageHandleStandardArgs)

# This macro puts all listed variables in the parent scope
MACRO(WAVE_SET_PARENT_SCOPE)
    FOREACH(varname ${ARGN})
        SET(${varname} ${${varname}} PARENT_SCOPE)
    ENDFOREACH()
ENDMACRO(WAVE_SET_PARENT_SCOPE)

# Define a function for new scope
# This isolates all variables inappropriately set by PCLConfig.cmake
FUNCTION(_find_pcl)
    FIND_PACKAGE(PCL ${PCL_FIND_VERSION} QUIET CONFIG
        COMPONENTS ${PCL_FIND_COMPONENTS})

    # PCL adds some entries with spaces instead of as a (semicolon-separated)
    # CMake list, which messes up later. Fix them here.
    STRING(REPLACE " " ";" pcl_definitions "${PCL_DEFINITIONS}")

    # This would get added directly to CMAKE_CXX_FLAGS, which we're avoiding
    LIST(APPEND pcl_definitions ${VTK_REQUIRED_CXX_FLAGS})

    # Export these variables directly to the parent scope
    WAVE_SET_PARENT_SCOPE(
        PCL_FOUND
        PCL_DIR
        PCL_CONFIG
        PCL_CONSIDERED_CONFIGS
        PCL_CONSIDERED_VERSIONS
        PCL_TO_FIND_COMPONENTS
        PCL_INCLUDE_DIRS
        PCL_LIBRARY_DIRS
        PCL_LIBRARIES)

    FOREACH(component ${PCL_FIND_COMPONENTS})
        WAVE_SET_PARENT_SCOPE(
            PCL_${component}_FOUND
            PCL_${component}_LIBRARY
            PCL_${component}_LIBRARIES
            PCL_${component}_INCLUDE_DIR
            PCL_${component}_DEFINITIONS)
    ENDFOREACH()

    # Set these variables with changes
    SET(PCL_VERSION ${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR} PARENT_SCOPE)
    SET(PCL_DEFINITIONS ${pcl_definitions} PARENT_SCOPE)

ENDFUNCTION(_find_pcl)
_find_pcl()

# Handle the QUIET and REQUIRED options
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PCL CONFIG_MODE)
