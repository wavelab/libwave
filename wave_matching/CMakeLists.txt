CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_matching)

find_package(PCL REQUIRED common registration point_types io)

add_definitions(${EIGEN_DEFINITIONS}
        ${PCL_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        /usr/include
        /usr/include/eigen3
        /usr/local/include
        include
        )

link_directories(${PCL_LIBRARY_DIRS}
        ${EIGEN_LIBRARY_DIRS}
        /usr/lib
        /usr/local/lib)

set(DEPS
    ${PCL_COMMON_LIBRARIES}
    ${PCL_REGISTRATION_LIBRARIES}
    ${PCL_POINT_TYPES_LIBRARIES}
    ${PCL_IO_LIBRARIES})

add_library(wave_matching STATIC src/icp.cpp)
target_link_libraries(wave_matching ${DEPS})

# Unit tests
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(
    # TARGET
    icp_tests
    # TESTS
    tests/icp_tests.cpp
    # TEST RUNNER
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(icp_tests
    wave_matching
    gtest
    ${CMAKE_THREAD_LIBS_INIT})