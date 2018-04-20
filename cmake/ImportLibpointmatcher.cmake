IF(libpointmatcher_INCLUDE_DIRS AND NOT TARGET libpointmatcher)
    ADD_LIBRARY(libpointmatcher INTERFACE IMPORTED)
    SET_PROPERTY(TARGET libpointmatcher PROPERTY
            INTERFACE_INCLUDE_DIRECTORIES "${libpointmatcher_INCLUDE_DIRS}")
    SET_PROPERTY(TARGET libpointmatcher PROPERTY
            INTERFACE_LINK_LIBRARIES "${libpointmatcher_LIBRARIES}")
ENDIF()