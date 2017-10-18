CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(aslam_cameras)

FIND_PACKAGE(Boost REQUIRED COMPONENTS system filesystem)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(include
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
ADD_LIBRARY(${PROJECT_NAME}
    STATIC
    src/CameraGeometryBase.cpp
    src/GlobalShutter.cpp
    src/RollingShutter.cpp
    src/NoMask.cpp
    src/NoDistortion.cpp
    src/RadialTangentialDistortion.cpp
    src/EquidistantDistortion.cpp
    src/FovDistortion.cpp
    src/ImageMask.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
ADD_DEFINITIONS(-DGTEST_USE_OWN_TR1_TUPLE=0)

WAVE_ADD_TEST(${PROJECT_NAME}_tests
    test/PinholeCameraGeometry.cpp
    test/OmniCameraGeometry.cpp
    test/RadialTangentialDistortion.cpp
    test/EquidistantDistortion.cpp
    test/FovDistortion.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    ${Boost_LIBRARIES})
