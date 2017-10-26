# - Defines an imported target for kindr
# This script is meant to be used after find_package(kindr)
# It adds the imported target Kindr::kindr if it doesn't already exist
IF(kindr_FOUND AND NOT TARGET Kindr::kindr)
    ADD_LIBRARY(Kindr::kindr INTERFACE IMPORTED)
    SET_PROPERTY(TARGET Kindr::kindr PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES "${kindr_INCLUDE_DIRS}")
ENDIF()
