project(wave_docs)

# DOCS_INPUT is a space-separated list of subdirectories, used for Doxygen input
set(WAVE_DOXY_INPUT
    "${WAVE_SOURCE_DIR}/README.md ${CMAKE_CURRENT_SOURCE_DIR}/ref")

foreach(dir ${LIBWAVE_MODULES})
    include_directories(${dir}/include)
    set(WAVE_DOXY_INPUT "${WAVE_DOXY_INPUT} ${WAVE_SOURCE_DIR}/${dir}")
endforeach()


find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

    # Find images under "ref". We must copy them separately
    file(GLOB_RECURSE REF_IMAGES ref "*.jpg" "*.png" "*.gif")

    # Generate documentation with Doxygen
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        COMMAND ${CMAKE_COMMAND} -E
            copy ${REF_IMAGES} ${WAVE_BINARY_DIR}/docs/html
        WORKING_DIRECTORY ${WAVE_SOURCE_DIR}/docs
        COMMENT "Generating API documentation with Doxygen" VERBATIM
        )



endif()


