# - Functions and macros used in libwave

INCLUDE(CMakeParseArguments)
INCLUDE(GNUInstallDirs)

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
    # Link wave_utils as that contains wave_test.hpp
    TARGET_LINK_LIBRARIES(${NAME} gtest gtest_main pthread wave::utils)

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

# wave_add_benchmark: Add a target which links against google benchmark
#
# WAVE_ADD_BENCHMARK(Name src1 [src2...])
#
# The target will be added to the tests run by "make benchmark". It will be
# linked against the needed libraries. Any other links can be made separately
# with the target_link_libraries command.
FUNCTION(WAVE_ADD_BENCHMARK NAME)
    # Build the executable using the given sources
    ADD_EXECUTABLE(${NAME} ${ARGN})

    TARGET_LINK_LIBRARIES(${NAME} benchmark::benchmark)

    # Put the executable in the benchmarks/ directory
    SET_TARGET_PROPERTIES(${NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/benchmarks)

    # Add a ctest that will not be run by default.
    # To run it the command `ctest -C benchmark -L benchmark` must be used.
    # (Setting -C excludes it from the default set, and setting -L excludes the
    # default tests from it). Note the target `benchmark` runs this command.
    ADD_TEST(NAME ${NAME} COMMAND ${NAME} CONFIGURATIONS benchmark)
    SET_TESTS_PROPERTIES(${NAME} PROPERTIES LABELS benchmark)
ENDFUNCTION(WAVE_ADD_BENCHMARK)

# wave_include_directories: Set a module's include paths so they are usable
# from both the build and install tree
#
# WAVE_INCLUDE_DIRECTORIES(TARGET <INTERFACE|PUBLIC> dir1 [dir2...])
#
# This function expects relative paths, typically "include". It wraps
# TARGET_INCLUDE_DIRECTORIES, using a generator expression for the includes.
#
# See:
# https://cmake.org/cmake/help/v3.4/manual/cmake-buildsystem.7.html#include-directories-and-usage-requirements
# https://stackoverflow.com/a/25681179
FUNCTION(WAVE_INCLUDE_DIRECTORIES TARGET MODE)
    FOREACH(path ${ARGN})
        TARGET_INCLUDE_DIRECTORIES(${TARGET} ${MODE}
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${path}>
            $<INSTALL_INTERFACE:${path}>)
    ENDFOREACH()
ENDFUNCTION(WAVE_INCLUDE_DIRECTORIES)

# wave_add_library: Adds a libwave component library, setting standard install
# destinations and properties
#
# WAVE_ADD_LIBRARY(Name [source1...])
#
# This convenience function wraps ADD_LIBRARY, adding either a normal library
# with the given sources or a header-only INTERFACE library if no sources are
# given.
#
# It calls INSTALL(...) and sets standard paths for binaries and header files,
# and associates the target with the waveTargets export.
#
# This function also sets the output library name to the target name prefixed
# with "wave_". For example, for the name "utils" the library built would be
# "libwave_utils.a".
FUNCTION(WAVE_ADD_LIBRARY NAME)

    IF(NOT ARGN)
        # No sources given - header-only library
        ADD_LIBRARY(${NAME} INTERFACE)
    ELSE()
        # Normal library, will be static unless BUILD_SHARED_LIBS is ON
        ADD_LIBRARY(${NAME} ${ARGN})
    ENDIF()

    # Remove "wave_" prefix from exported name
    # This means others can link to wave::utils instead of wave::wave_utils
    STRING(REGEX REPLACE "^wave_" "" unprefixed_name ${NAME})
    SET_TARGET_PROPERTIES(${NAME} PROPERTIES EXPORT_NAME ${unprefixed_name})

    # Add an alias so internal target_link_libraries can also use the namespace
    # The benefit is newer CMake will give immediately give an error on typos
    ADD_LIBRARY(wave::${unprefixed_name} ALIAS ${NAME})

    # Add this target to the all-inclusive "wave" library
    SET_PROPERTY(TARGET wave APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${NAME})

    # Add install destinations
    # The variables used here are defined by GNUInstallDirs
    INSTALL(TARGETS ${NAME} EXPORT waveTargets
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
ENDFUNCTION(WAVE_ADD_LIBRARY)
