# - Defines an imported target for PCL
# This script is meant to be used after find_package(PCL COMPONENTS ...)
# It adds a monolithic imported target PCL::PCL, if it doesn't already exist,
# and targets PCL::<component> for each requested component


# This macro takes a list such as
#     a;b;optimized;c;debug;d;e
# and turns it into
#     a;b;c;e
# It is a workaround for PCLConfig setting "debug" and "optimized" libraries
# which cannot be used for the INTERFACE_LINK_LIBRARIES property.
# In practice the "debug" and "optimized" items are the same.
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
    SET(imported_pcl_libraries "")

    FOREACH(component ${PCL_TO_FIND_COMPONENTS})
        STRING(TOUPPER "${component}" ucomponent)
        SET(component_libs "${PCL_${ucomponent}_LIBRARY}"
            "${PCL_${ucomponent}_LIBRARIES}")
        WAVE_REMOVE_DEBUG_FROM_LIBRARY_LIST(component_libs)
        LIST(REMOVE_DUPLICATES component_libs)
        LIST(APPEND imported_pcl_libraries ${component_libs})


        IF(NOT TARGET PCL::${component})
            ADD_LIBRARY(PCL::${component} INTERFACE IMPORTED)
            SET_TARGET_PROPERTIES(PCL::${component} PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${PCL_${ucomponent}_INCLUDE_DIR}"
                INTERFACE_LINK_LIBRARIES "component_libs"
                INTERFACE_COMPILE_OPTIONS "${PCL_${ucomponent}_DEFINITIONS}")
        ENDIF()
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES imported_pcl_libraries)

    # PCL adds some entries with spaces instead of as a (semicolon-separated)
    # CMake list, which messes up later. Fix them here.
    STRING(REPLACE " " ";" pcl_definitions "${PCL_DEFINITIONS}")

    IF(NOT TARGET PCL::PCL)
        # Add monolithic target with all components requested from PCL
        ADD_LIBRARY(PCL::PCL INTERFACE IMPORTED)
        SET_TARGET_PROPERTIES(PCL::PCL  PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${imported_pcl_libraries}"
            INTERFACE_COMPILE_OPTIONS "${pcl_definitions}")
    ENDIF()
ENDIF()
