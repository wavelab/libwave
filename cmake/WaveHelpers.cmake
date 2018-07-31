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
    TARGET_LINK_LIBRARIES(${NAME} gtest gtest_main wave::utils)

    # Put the test executable in the tests/ directory
    SET_TARGET_PROPERTIES(${NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

    # Build this test on "make tests"
    ADD_DEPENDENCIES(tests ${NAME})

    IF(NOT WAVE_ADD_TEST_DISABLED)
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

    # Build this target on "make benchmarks"
    ADD_DEPENDENCIES(benchmarks ${NAME})
ENDFUNCTION(WAVE_ADD_BENCHMARK)

# wave_include_directories: Set a module's public include paths so they are
# usable from both the build and install tree
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
        INSTALL(DIRECTORY ${path}/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
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
    # Note INCLUDES DESTINATION does not actually install anything; it only sets
    # paths on the exported targets. Due to this quirk we also call INSTALL in
    # WAVE_INCLUDE_DIRECTORIES.
ENDFUNCTION(WAVE_ADD_LIBRARY)


# wave_check_module: Declares an optional libwave component library, and
# returns from the script if it cannot or should not be built.
#
# WAVE_CHECK_MODULE(Name [DEPENDS target1...])
#
# This macro is meant to be called from the CMakeLists script defining the
# libwave component. It:
#  - Adds a user option BUILD_name, which is ON by default but can be disabled
#  - Checks that each target listed after DEPENDS exists
#
# If the user option is disabled or one of the DEPENDS does not exist, this
# macro returns from the calling script (returning to the top-level CMake
# assuming it was called from a subdirectory). In either case, it prints a
# message whether the component is being built, and the reason if not.
#
# Note this is a macro (which does not have its own scope), so RETURN() exits
# the calling script.
MACRO(WAVE_CHECK_MODULE NAME)

    # Define the arguments this macro accepts
    SET(options "")
    SET(one_value_args "")
    SET(multi_value_args "DEPENDS")
    CMAKE_PARSE_ARGUMENTS(WAVE_COMPONENT
        "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    # Add a user option
    OPTION(BUILD_${NAME} "Build the ${NAME} library" ON)

    IF(NOT ${BUILD_${NAME}})
        MESSAGE(STATUS "Not building ${NAME}: Disabled by user")
        RETURN()
    ENDIF()

    # Check that each of the given DEPENDS (if any) exists as a target
    FOREACH(dep IN LISTS WAVE_COMPONENT_DEPENDS)
        IF(NOT TARGET ${dep})
            MESSAGE(STATUS "Not building ${NAME}: Requires ${dep}")
            RETURN()
        ENDIF()
    ENDFOREACH()

    MESSAGE(STATUS "Building ${NAME}")

ENDMACRO(WAVE_CHECK_MODULE)

# wave_add_module: Does everything needed to add a typical libwave component
# library.
#
# WAVE_ADD_MODULE(Name
#                [DEPENDS target1...]
#                [SOURCES source1...])
#
# This macro does the equivalent of the following:
#
# WAVE_CHECK_MODULE(Name DEPENDS <depends>)
# WAVE_ADD_LIBRARY(Name <sources>)
# WAVE_INCLUDE_DIRECTORIES(Name PUBLIC "include")
# TARGET_LINK_LIBRARIES(Name PUBLIC <depends>)
#
# If no SOURCES are given (for a header-only library) INTERFACE is used instead
# of PUBLIC.
MACRO(WAVE_ADD_MODULE NAME)

    # Define the arguments this macro accepts
    SET(options "")
    SET(one_value_args "")
    SET(multi_value_args DEPENDS SOURCES)
    CMAKE_PARSE_ARGUMENTS(WAVE_ADD_MODULE
        "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    # Check if the module should be built, based on options and dependencies
    WAVE_CHECK_MODULE(${NAME} DEPENDS ${WAVE_ADD_MODULE_DEPENDS})

    IF(WAVE_ADD_MODULE_SOURCES)
        SET(link_type PUBLIC)
    ELSE()
        # No sources given - header-only library
        SET(link_type INTERFACE)
    ENDIF()

    WAVE_ADD_LIBRARY(${NAME} ${WAVE_ADD_MODULE_SOURCES})

    # Use these headers when building, and make clients use them
    WAVE_INCLUDE_DIRECTORIES(${NAME} ${link_type} "include")

    # Depend on these modules and external libraries, and make clients use them
    TARGET_LINK_LIBRARIES(${NAME} ${link_type} ${WAVE_ADD_MODULE_DEPENDS})
ENDMACRO(WAVE_ADD_MODULE)
