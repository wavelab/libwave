CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave)

# COMPILER SETTINGS
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++11)
ENDIF()

# CMAKE SETTINGS
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
INCLUDE(cmake/WaveUtils.cmake)
ENABLE_TESTING()


# INCLUDES
INCLUDE_DIRECTORIES(
  wave_controls/include
  wave_geometry/include
  wave_kinematics/include
  wave_matching/include
  wave_optimization/include
  wave_utils/include
  wave_containers/include
  wave_benchmark/include
)

# gtest
ADD_SUBDIRECTORY(deps/googletest)
INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include/ ${gtest_SOURCE_DIR})

# libwave_controls.a
ADD_SUBDIRECTORY(wave_controls)
ADD_DEPENDENCIES(wave_controls wave_utils)

# libwave_geometry.a
ADD_SUBDIRECTORY(wave_geometry)
ADD_DEPENDENCIES(wave_geometry wave_utils)

# libwave_kinematics.a
ADD_SUBDIRECTORY(wave_kinematics)
ADD_DEPENDENCIES(wave_kinematics wave_controls wave_utils)

# libwave_matching.a
ADD_SUBDIRECTORY(wave_matching)
ADD_DEPENDENCIES(wave_matching wave_utils)

# libwave_optimization.a
ADD_SUBDIRECTORY(wave_optimization)
ADD_DEPENDENCIES(wave_optimization wave_utils)

# libwave_utils.a
ADD_SUBDIRECTORY(wave_utils)

# wave_containers: header-only library
ADD_SUBDIRECTORY(wave_containers)

ADD_SUBDIRECTORY(wave_benchmark)
ADD_DEPENDENCIES(wave_benchmark wave_utils)

# libwave library
SET(LIBWAVE_MODULE_LIBS
    ${CMAKE_BINARY_DIR}/wave_controls/libwave_controls.a
    ${CMAKE_BINARY_DIR}/wave_geometry/libwave_geometry.a
    ${CMAKE_BINARY_DIR}/wave_kinematics/libwave_kinematics.a
    ${CMAKE_BINARY_DIR}/wave_matching/libwave_matching.a
    ${CMAKE_BINARY_DIR}/wave_optimization/libwave_optimization.a
    ${CMAKE_BINARY_DIR}/wave_utils/libwave_utils.a)

SET(LIBWAVE_MODULES
    wave_controls
    wave_geometry
    wave_kinematics
    wave_matching
    wave_optimization
    wave_utils
    wave_benchmark)

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
    INSTALL(FILES ${CMAKE_BINARY_DIR}/libwave.a DESTINATION CMAKE_INSTALL_PREFIX}/lib)

ENDIF()

# libwave headers
FOREACH(MODULE ${LIBWAVE_MODULES})
  INSTALL(DIRECTORY ${MODULE}/include/ DESTINATION include)
ENDFOREACH()
