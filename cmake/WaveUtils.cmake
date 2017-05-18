# - Macros used in libwave
#
# wave_add_test: Add a gtest target
#
# WAVE_ADD_TEST(Name [DISABLED] src1 [src2...])
#
# The test will be added to the tests run by "make test", unless DISABLED is
# given. It will be linked against the needed gtest libraries. Any other links
# can be made separately with the target_link_libraries command.
function(wave_add_test NAME)

    # Define the arguments this function accepts
    set(options DISABLED)
    set(one_value_args "")
    set(multi_value_args "")
    cmake_parse_arguments(WAVE_ADD_TEST
        "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    # Build the test executable using the given sources
    add_executable(${NAME} ${WAVE_ADD_TEST_UNPARSED_ARGUMENTS})

    # Link gtest libraries including one providing main()
    target_link_libraries(${NAME} gtest gtest_main pthread)

    # Put the test executable in the tests/ directory
    set_target_properties(${NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

    if(WAVE_ADD_TEST_DISABLED)
        message(WARNING "Test ${NAME} is disabled: building, but not adding it to tests")
    else()
        message(STATUS "Adding test ${NAME}")
        # Add the executable as a test, so it runs with "make test"
        add_test(NAME ${NAME} COMMAND ${NAME})
    endif()

endfunction(wave_add_test)
