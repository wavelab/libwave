CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_vision)

# PACKAGES
FIND_PACKAGE(OpenCV REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

# LIBRARY DIRS
LINK_DIRECTORIES(${EIGEN_LIBRARY_DIRS})
SET(
    WAVE_VISION_DEPS
    yaml-cpp
    wave_utils
    ${OpenCV_LIBS}
)

SET(
    WAVE_VISION_TEST_DEPS
    yaml-cpp
    wave_utils
    gtest
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)
ADD_LIBRARY(
    wave_vision
    STATIC
    src/fast_detector.cpp)
TARGET_LINK_LIBRARIES(wave_vision ${WAVE_VISION_DEPS})

# UNIT TESTS
# SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
# FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
# FILE(COPY tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
# ADD_EXECUTABLE(
#     # TARGET
#     wave_vision_tests
#     # TESTS
#     # TEST RUNNER
#     tests/test_runner.cpp
# )
# TARGET_LINK_LIBRARIES(wave_vision_tests ${WAVE_VISION_TEST_DEPS})
