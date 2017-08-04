CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave)

SET(CMAKE_BUILD_TYPE RELEASE)

# COMPILER SETTINGS
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# CMAKE SETTINGS
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
INCLUDE(cmake/WaveUtils.cmake)
ENABLE_TESTING()


# The sub-libraries, each of which is in a subdirectory of the same name
SET(LIBWAVE_MODULES
    wave_controls
    wave_containers
    wave_geometry
    wave_kinematics
    wave_matching
    wave_optimization
    wave_vision
    wave_utils
    wave_benchmark)

FOREACH(dir ${LIBWAVE_MODULES})
    INCLUDE_DIRECTORIES(${dir}/include)
ENDFOREACH()
FOREACH(dir ${LIBWAVE_MODULES})
    ADD_SUBDIRECTORY(${dir})
ENDFOREACH()

# gtest
ADD_SUBDIRECTORY(deps/googletest)
INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include/ ${gtest_SOURCE_DIR})

# Define dependencies between modules
# libwave_controls.a
ADD_DEPENDENCIES(wave_controls wave_utils)

# libwave_geometry.a
ADD_DEPENDENCIES(wave_geometry wave_utils)

# libwave_kinematics.a
ADD_DEPENDENCIES(wave_kinematics wave_controls wave_utils)

# libwave_matching.a
ADD_DEPENDENCIES(wave_matching wave_utils)

# libwave_optimization.a
ADD_DEPENDENCIES(wave_optimization wave_utils)

# libwave_vision.a
ADD_DEPENDENCIES(wave_vision wave_utils)

# libwave_benchmark.a
ADD_DEPENDENCIES(wave_benchmark wave_utils)

# libwave library
SET(LIBWAVE_MODULE_LIBS
    ${CMAKE_BINARY_DIR}/wave_controls/libwave_controls.a
    ${CMAKE_BINARY_DIR}/wave_geometry/libwave_geometry.a
    ${CMAKE_BINARY_DIR}/wave_kinematics/libwave_kinematics.a
    ${CMAKE_BINARY_DIR}/wave_matching/libwave_matching.a
    ${CMAKE_BINARY_DIR}/wave_optimization/libwave_optimization.a
    ${CMAKE_BINARY_DIR}/wave_utils/libwave_utils.a
    ${CMAKE_BINARY_DIR}/wave_vision/libwave_vision.a)

IF(BUILD_SHARED_LIBS)
    # SHARED LIBRARY
    ADD_CUSTOM_TARGET(
        libwave ALL
        COMMAND
            gcc -shared -o libwave.so
              -Wl,--whole-archive
                ${LIBWAVE_MODULE_LIBS}
              -Wl,--no-whole-archive
        DEPENDS
            ${LIBWAVE_MODULES}
    )
    INSTALL(FILES ${CMAKE_BINARY_DIR}/libwave.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

ELSE ()
    # STATIC LIBRARY
    ADD_CUSTOM_TARGET(
        libwave ALL
        COMMAND
            ar -rcT libwave.a ${LIBWAVE_MODULE_LIBS}
        DEPENDS
            ${LIBWAVE_MODULES}
    )
    INSTALL(FILES ${CMAKE_BINARY_DIR}/libwave.a DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

ENDIF()

# libwave headers
FOREACH(MODULE ${LIBWAVE_MODULES})
  INSTALL(DIRECTORY ${MODULE}/include/ DESTINATION include)
ENDFOREACH()

# Documentation
SET(WAVE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(WAVE_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
ADD_SUBDIRECTORY(docs)
