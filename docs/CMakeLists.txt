CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)
PROJECT(wave_docs)

# DOCS_INPUT is a space-separated list of subdirectories, used for Doxygen input
SET(WAVE_DOXY_INPUT
    "${WAVE_SOURCE_DIR}/README.md ${CMAKE_CURRENT_SOURCE_DIR}/ref")

FOREACH(dir ${LIBWAVE_MODULES})
    INCLUDE_DIRECTORIES(${dir}/include)
    SET(WAVE_DOXY_INPUT "${WAVE_DOXY_INPUT} ${WAVE_SOURCE_DIR}/${dir}")
ENDFOREACH()


FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
    CONFIGURE_FILE(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

    # Find images under "ref". We must copy them separately
    FILE(GLOB_RECURSE REF_IMAGES ref "*.jpg" "*.png" "*.gif")

    # Generate documentation with Doxygen
    ADD_CUSTOM_TARGET(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        COMMAND ${CMAKE_COMMAND} -E
            copy ${REF_IMAGES} ${WAVE_BINARY_DIR}/docs/html
        WORKING_DIRECTORY ${WAVE_SOURCE_DIR}/docs
        COMMENT "Generating API documentation with Doxygen" VERBATIM)
ENDIF()
