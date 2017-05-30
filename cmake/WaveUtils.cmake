# - Macros used in libwave

include(CMakeParseArguments)

# wave_add_test: Add a gtest target
#
# WAVE_ADD_TEST(Name [DISABLED] src1 [src2...])
#
# The test will be added to the tests run by "make test", unless DISABLED is
# given. It will be linked against the needed gtest libraries. Any other links
# can be made separately with the target_link_libraries command.
FUNCTION(WAVE_ADD_TEST NAME)

    # Define the arguments this function accepts
    SET(options DISABLED)
    SET(one_value_args "")
    SET(multi_value_args "")
    CMAKE_PARSE_ARGUMENTS(WAVE_ADD_TEST
        "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    # Build the test executable using the given sources
    ADD_EXECUTABLE(${NAME} ${WAVE_ADD_TEST_UNPARSED_ARGUMENTS})

    # Link gtest libraries including one providing main()
    TARGET_LINK_LIBRARIES(${NAME} gtest gtest_main pthread)

    # Put the test executable in the tests/ directory
    SET_TARGET_PROPERTIES(${NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

    IF(WAVE_ADD_TEST_DISABLED)
        MESSAGE(WARNING "Test ${NAME} is disabled: building, but not adding it to tests")
    ELSE()
        MESSAGE(STATUS "Adding test ${NAME}")
        # Add the executable as a test, so it runs with "make test"
        ADD_TEST(NAME ${NAME} COMMAND ${NAME})
    ENDIF()

ENDFUNCTION(WAVE_ADD_TEST)
