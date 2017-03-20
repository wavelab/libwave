CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_matching)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++11)
ENDIF()

find_package(PCL 1.8 REQUIRED common point_types io)
find_package(Eigen3 REQUIRED core)
find_package(cartographer REQUIRED)
find_package(GTest REQUIRED)

set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)

add_definitions(${EIGEN_DEFINITIONS}
        ${PCL_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        /usr/include
        /usr/local/include
        /usr/include/lua5.2
        include
        )

link_directories(${PCL_LIBRARY_DIRS}
        ${EIGEN_LIBRARY_DIRS}
        /usr/lib
        /usr/local/lib)

set(DEPS
    cartographer
    ${PCL_COMMON_LIBRARIES}
    ${PCL_POINT_TYPES_LIBRARIES})

add_library(wave_matching
    STATIC
    src/correlation.cpp
    src/fast_correlation.cpp
    src/ceres_matcher.cpp)

target_link_libraries(wave_matching ${DEPS})

# unit tests
add_executable(corr_tests tests/corr_tests.cpp)
target_link_libraries(corr_tests wave_matching ${PCL_IO_LIBRARIES} ${GTEST_LIBRARIES})

add_executable(fast_corr_tests tests/fast_corr_tests.cpp)
target_link_libraries(fast_corr_tests wave_matching ${PCL_IO_LIBRARIES} ${GTEST_LIBRARIES})

add_executable(ceres_tests tests/ceres_tests.cpp)
target_link_libraries(ceres_tests wave_matching ${PCL_IO_LIBRARIES} ${GTEST_LIBRARIES})
