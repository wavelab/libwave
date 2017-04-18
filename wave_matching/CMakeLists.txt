CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_matching)

SET( CMAKE_BUILD_TYPE release )

FIND_PACKAGE(PCL REQUIRED common registration point_types io)
FIND_PACKAGE(OpenCV REQUIRED)

ADD_DEFINITIONS(${EIGEN_DEFINITIONS}
        ${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        /usr/include
        /usr/include/eigen3
        /usr/local/include
        include
        )

LINK_DIRECTORIES(${PCL_LIBRARY_DIRS}
        ${EIGEN_LIBRARY_DIRS}
        /usr/lib
        /usr/local/lib
        /usr/lib/x86_64-linux-gnu/)

SET(DEPS
    yaml-cpp
    wave_utils
    ${OpenCV_LIBS}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_REGISTRATION_LIBRARIES}
    ${PCL_POINT_TYPES_LIBRARIES}
    ${PCL_IO_LIBRARIES})

SET(TEST_DEPS
    wave_matching
    wave_utils
    gtest
    ${CMAKE_THREAD_LIBS_INIT})

ADD_LIBRARY(wave_matching STATIC src/icp.cpp src/ndt.cpp)
TARGET_LINK_LIBRARIES(wave_matching ${DEPS})

# Unit tests
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY config DESTINATION ${PROJECT_BINARY_DIR}/tests)

ADD_EXECUTABLE(
    # TARGET
    registration_tests
    # TESTS
    tests/icp_tests.cpp
    tests/ndt_tests.cpp
    # TEST RUNNER
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(registration_tests
    ${TEST_DEPS})
