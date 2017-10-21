# - Try to find PCL library
# This module uses find_package(PCL), then defines IMPORTED targets
#
# Once done this will define PCL, a monolithic target linking to all
# PCL_LIBRARIES and PCL_INCLUDE_DIRS
#
# @todo PCL's config file is messy and outdated (overwriting BOOST_LIBRARIES,
# for example), thus this is messy. (Maybe there is a better solution).


# Define a function for new scope
# This hides all variables set by PCLConfig.cmake, such as BOOST_LIBRARIES
FUNCTION(_find_pcl)
    # Find PCL version and components used by libwave
    SET(wave_pcl_components
        common filters registration kdtree search io visualization)
    FIND_PACKAGE(PCL 1.8 REQUIRED ${wave_pcl_components})

    ADD_LIBRARY(PCL INTERFACE IMPORTED)

    # We don't use the given PCL_LIBRARIES since it includes "debug" and "optimized"
    # keywords; instead, we specify individual modules.
    SET(imported_pcl_libraries "")

    FOREACH(component ${wave_pcl_components})
        STRING(TOUPPER "${component}" component)
        LIST(APPEND imported_pcl_libraries ${PCL_${component}_LIBRARY})
    ENDFOREACH()

    # PCLConfig.cmake is evil for setting these
    LIST(APPEND imported_pcl_libraries
        ${BOOST_LIBRARIES}
        ${VTK_LIBRARIES})

    # PCL adds some entries with spaces instead of as a (semicolon-separated) CMake
    # list, which messes up later. Fix them here.
    string(REPLACE " " ";" pcl_definitions "${PCL_DEFINITIONS}")

    SET_TARGET_PROPERTIES(PCL PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${imported_pcl_libraries}"
        INTERFACE_COMPILE_OPTIONS "${pcl_definitions}")
ENDFUNCTION(_find_pcl)
_find_pcl()
