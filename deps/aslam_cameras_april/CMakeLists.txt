cmake_minimum_required(VERSION 2.8.3)
project(aslam_cameras_april)

FIND_PACKAGE(Boost REQUIRED COMPONENTS serialization)
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

file(GLOB logging_src
    "src/logging/*.cpp"
)

ADD_LIBRARY(${PROJECT_NAME}
    STATIC
    src/GridCalibrationTargetBase.cpp
    src/GridCalibrationTargetAprilgrid.cpp
    ${apriltags_src}
    ${logging_src}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_LIBRARIES})
