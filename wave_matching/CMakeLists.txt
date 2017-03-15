CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_matching)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++11)
ENDIF()

find_package(PCL 1.8 REQUIRED common registration point_types io)
find_package(Eigen3 REQUIRED core)
find_package(GTest REQUIRED)

set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)

add_definitions(${EIGEN_DEFINITIONS}
        ${PCL_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        /usr/include
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
    ${PCL_POINT_TYPES_LIBRARIES})

add_library(wave_matching SHARED src/ICP.cpp)
target_link_libraries(wave_matching ${DEPS})
