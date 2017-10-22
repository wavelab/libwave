# - Try to find PCL library
# This module wraps find_package(PCL CONFIG) with two changes in what it defines
#
# * It doesn't overwrite variables it does not own such as BOOST_LIBRARIES and
#   CMAKE_CXX_FLAGS.
# * The variable PCL_LIBRARIES is limited to libraries for the requested
#   components
#
# @author Leo Koppel <lkoppel@uwaterloo.ca>

INCLUDE(FindPackageHandleStandardArgs)

# Define a function for new scope
# This isolates all variables inappropriately set by PCLConfig.cmake
FUNCTION(_find_pcl)
    FIND_PACKAGE(PCL ${PCL_FIND_VERSION} QUIET CONFIG
        COMPONENTS ${PCL_FIND_COMPONENTS})

    # We don't use the given PCL_LIBRARIES since it includes "debug" and "optimized"
    # keywords; instead, we specify individual modules.
    SET(requested_pcl_libraries "")

    FOREACH(component ${PCL_FIND_COMPONENTS})
        STRING(TOUPPER "${component}" component)
        LIST(APPEND requested_pcl_libraries ${PCL_${component}_LIBRARY})

        # Allow these out of the scope
        foreach(var FOUND LIBRARY LIBRARIES INCLUDE_DIR DEFINITIONS)
            SET(PCL_${component}_${var} ${PCL_${component}_${var}} PARENT_SCOPE)
        endforeach()
    ENDFOREACH()

    # PCLConfig.cmake is evil for setting these
    LIST(APPEND requested_pcl_libraries
        ${BOOST_LIBRARIES}
        ${VTK_LIBRARIES})

    # This gets added directly to CMAKE_CXX_FLAGS, which we're avoiding
    LIST(APPEND PCL_DEFINITIONS ${VTK_REQUIRED_CXX_FLAGS})

    # Export these variables to the parent scope
    SET(PCL_FOUND ${PCL_FOUND} PARENT_SCOPE)
    SET(PCL_DIR ${PCL_DIR} PARENT_SCOPE)
    SET(PCL_CONFIG ${PCL_CONFIG} PARENT_SCOPE)
    SET(PCL_CONSIDERED_CONFIGS ${PCL_CONSIDERED_CONFIGS} PARENT_SCOPE)
    SET(PCL_CONSIDERED_VERSIONS ${PCL_CONSIDERED_VERSIONS} PARENT_SCOPE)
    SET(PCL_TO_FIND_COMPONENTS ${PCL_TO_FIND_COMPONENTS} PARENT_SCOPE)


    # Note we only use the subset of libraries for the requested components
    SET(PCL_LIBRARIES ${requested_pcl_libraries} PARENT_SCOPE)
    SET(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIRS} PARENT_SCOPE)
    SET(PCL_LIBRARY_DIRS ${PCL_LIBRARY_DIRS} PARENT_SCOPE)
    SET(PCL_DEFINITIONS ${PCL_DEFINITIONS} PARENT_SCOPE)
    SET(PCL_VERSION ${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR} PARENT_SCOPE)
ENDFUNCTION(_find_pcl)
_find_pcl()

# Handle the QUIET and REQUIRED options
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PCL CONFIG_MODE)
