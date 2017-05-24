CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_vision)

# DEPENDENCIES
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# DEFINITIONS
ADD_DEFINITIONS(${OpenCV_DEFINITIONS})

# INCLUDES
INCLUDE_DIRECTORIES(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
)

# LIBRARY
SET(
    WAVE_VISION_DEPS
    yaml-cpp
    wave_utils
    ${OpenCV_LIBS}
)
ADD_LIBRARY(
    wave_vision
    STATIC
    src/fast_detector.cpp
)
TARGET_LINK_LIBRARIES(wave_vision ${WAVE_VISION_DEPS})

# UNIT TESTS
WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/fast_tests.cpp)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_utils
)
# Copy test data
file(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)

