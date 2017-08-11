cmake_minimum_required(VERSION 2.8.3)
project(aslam_cameras_april)

FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem regex system serialization thread)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

include_directories(include
    include/sm
    include/apriltags
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS})

file(GLOB apriltags_src
    "src/apriltags/*.cc"
)

file(GLOB sm_src
    "src/sm/logging/*.cpp"
)

ADD_LIBRARY(${PROJECT_NAME}
    STATIC
    src/GridCalibrationTargetBase.cpp
    src/GridCalibrationTargetAprilgrid.cpp
    ${apriltags_src}
    ${sm_src}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_LIBRARIES})

WAVE_ADD_TEST(${PROJECT_NAME}_tests
    test/eigen/EigenSerializationTests.cpp
    test/logging/logTest.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
)
