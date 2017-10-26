# - Defines an imported target for PCL
# This script is meant to be used after find_package(PCL COMPONENTS ...)
# It adds a monolithic imported target PCL::PCL
#
# Note this script does *not* assume our own FindPCL module was used.
#
# @author Leo Koppel <lkoppel@uwaterloo.ca>


# This macro takes a list such as
#     a;b;optimized;c;debug;d;e
# and turns it into
#     a;b;c;e
# It is a workaround for PCLConfig setting "debug" and "optimized" libraries
# which cannot be used for the INTERFACE_LINK_LIBRARIES property.
# In practice the "debug" and "optimized" items are (usually?) the same.
MACRO(WAVE_REMOVE_DEBUG_FROM_LIBRARY_LIST LISTNAME)
    SET(index 0)
    WHILE(TRUE)
        LIST(FIND ${LISTNAME} debug index)
        IF(index LESS 0)
            BREAK()
        ENDIF()
        # Remove the "debug" and the following item
        LIST(REMOVE_AT ${LISTNAME} ${index})
        LIST(REMOVE_AT ${LISTNAME} ${index})
    ENDWHILE()
    LIST(REMOVE_ITEM ${LISTNAME} "optimized")
ENDMACRO()


IF(PCL_FOUND)
    # PCL adds some entries with spaces instead of as a (semicolon-separated)
    # CMake list, which messes up later. Fix them here.
    STRING(REPLACE " " ";" pcl_definitions "${PCL_DEFINITIONS}")

    # Remove repeated "optimized" and "debug" libraries
    WAVE_REMOVE_DEBUG_FROM_LIBRARY_LIST(PCL_LIBRARIES)

    # Add monolithic target with all libraries available from PCL
    IF(NOT TARGET PCL::PCL)
        ADD_LIBRARY(PCL::PCL INTERFACE IMPORTED)
        SET_TARGET_PROPERTIES(PCL::PCL  PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${PCL_LIBRARIES}"
            INTERFACE_COMPILE_OPTIONS "${pcl_definitions}")
    ENDIF()
ENDIF()
