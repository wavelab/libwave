CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam_vision)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# PACKAGES
FIND_PACKAGE(GTest REQUIRED)


# INCLUDES
INCLUDE_DIRECTORIES(
    /usr/include/
    /usr/include/eigen3
    /usr/local/include
    /usr/local/opencv3/include
    include
)


# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
    /usr/local/opencv3/lib
)


# OPENCV3
SET(OPENCV3_PATH /usr/local/opencv3)
SET(
    OPENCV3_LIBS
    ${OPENCV3_PATH}/lib/libopencv_calib3d.so
    ${OPENCV3_PATH}/lib/libopencv_core.so
    ${OPENCV3_PATH}/lib/libopencv_features2d.so
    ${OPENCV3_PATH}/lib/libopencv_flann.so
    ${OPENCV3_PATH}/lib/libopencv_highgui.so
    ${OPENCV3_PATH}/lib/libopencv_imgcodecs.so
    ${OPENCV3_PATH}/lib/libopencv_imgproc.so
    ${OPENCV3_PATH}/lib/libopencv_ml.so
    ${OPENCV3_PATH}/lib/libopencv_objdetect.so
    ${OPENCV3_PATH}/lib/libopencv_photo.so
    ${OPENCV3_PATH}/lib/libopencv_shape.so
    ${OPENCV3_PATH}/lib/libopencv_stitching.so
    ${OPENCV3_PATH}/lib/libopencv_superres.so
    ${OPENCV3_PATH}/lib/libopencv_ts.a
    ${OPENCV3_PATH}/lib/libopencv_video.so
    ${OPENCV3_PATH}/lib/libopencv_videoio.so
    ${OPENCV3_PATH}/lib/libopencv_videostab.so
    ${OPENCV3_PATH}/lib/libopencv_viz.so
)


# LIBRARY
ADD_LIBRARY(
    slam_vision
    STATIC
    src/calibration.cpp
    src/camera.cpp
    src/chessboard.cpp
    src/eight_point.cpp
    src/fast.cpp
    src/good.cpp
    src/orb.cpp
    src/sfm.cpp
    src/utils.cpp
    src/vo.cpp
)
SET(
    DEPS
    slam_vision
    slam_utils
    yaml-cpp
    ${GTEST_LIBRARIES}
    pthread
    ${OPENCV3_LIBS}
)


# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY scripts DESTINATION ${PROJECT_BINARY_DIR})

ADD_EXECUTABLE(calibration_test tests/calibration_test.cpp)
TARGET_LINK_LIBRARIES(calibration_test ${DEPS})

ADD_EXECUTABLE(camera_test tests/camera_test.cpp)
TARGET_LINK_LIBRARIES(camera_test ${DEPS})

ADD_EXECUTABLE(chessboard_test tests/chessboard_test.cpp)
TARGET_LINK_LIBRARIES(chessboard_test ${DEPS})

ADD_EXECUTABLE(eight_point_test tests/eight_point_test.cpp)
TARGET_LINK_LIBRARIES(eight_point_test ${DEPS})

ADD_EXECUTABLE(sfm_test tests/sfm_test.cpp)
TARGET_LINK_LIBRARIES(sfm_test ${DEPS})

ADD_EXECUTABLE(vo_test tests/vo_test.cpp)
TARGET_LINK_LIBRARIES(vo_test ${DEPS})


# UTILS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/utils)
FILE(COPY utils/data DESTINATION ${PROJECT_BINARY_DIR}/utils)

ADD_EXECUTABLE(calibration utils/calibration.cpp)
TARGET_LINK_LIBRARIES(calibration ${DEPS})

ADD_EXECUTABLE(feature_demo utils/feature_demo.cpp)
TARGET_LINK_LIBRARIES(feature_demo ${DEPS})

ADD_EXECUTABLE(optical_flow utils/optical_flow.cpp)
TARGET_LINK_LIBRARIES(optical_flow ${DEPS})

ADD_EXECUTABLE(video_capture utils/video_capture.cpp)
TARGET_LINK_LIBRARIES(video_capture ${DEPS})
