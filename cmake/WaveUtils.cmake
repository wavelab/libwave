function(wave_add_test NAME)
    set(options "")
    set(one_value_args "")
    set(multi_value_args "")
    cmake_parse_arguments(WAVE_ADD_TEST
        "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    message(STATUS "Adding test ${NAME}")

    # Build the test executable using the given sources
    add_executable(${NAME} ${WAVE_ADD_TEST_UNPARSED_ARGUMENTS})

    # Link gtest libraries including one providing main()
    target_link_libraries(${NAME} gtest gtest_main pthread)

    # Put the test executable in the tests/ directory
    set_target_properties(${NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

    # Add the executable as a test, so it runs with "make test"
    add_test(NAME ${NAME} COMMAND ${NAME})

endfunction(wave_add_test)