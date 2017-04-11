CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_vision)

# DEPENDENCIES
FIND_PACKAGE(OpenCV REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(/usr/include/eigen3)

# LIBRARY
ADD_LIBRARY(
    wave_vision
    STATIC
    src/dataset.cpp
    src/utils.cpp
)
SET(
  WAVE_OPENCV_DEPS
	opencv_calib3d
	opencv_core
	opencv_features2d
	opencv_flann
	opencv_highgui
	opencv_imgcodecs
	opencv_imgproc
	opencv_ml
	opencv_objdetect
	opencv_photo
	opencv_shape
	opencv_stitching
	opencv_superres
	opencv_video
	opencv_videoio
	opencv_videostab
)
SET(
    TEST_DEPS
    wave_vision
    wave_utils
    yaml-cpp
    ${WAVE_OPENCV_DEPS}
    gtest
    pthread
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
ADD_EXECUTABLE(
    # TARGET
    wave_vision_tests
    # TESTS
    tests/dataset_test.cpp
    # TEST RUNNER
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(wave_vision_tests ${TEST_DEPS})
