# - Defines an imported target for libnabo
# This script is meant to be used after find_package(libnabo)
# It adds the imported target libnabo::libnabo if it doesn't already exist
IF(libnabo_LIBRARIES AND NOT TARGET libnabo::libnabo)
    ADD_LIBRARY(libnabo::libnabo INTERFACE IMPORTED)
    SET_TARGET_PROPERTIES(libnabo::libnabo PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${libnabo_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${libnabo_LIBRARIES}")
ENDIF()
