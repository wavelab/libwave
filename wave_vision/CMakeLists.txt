CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_vision)

# DEPENDENCIES
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Boost 1.40 COMPONENTS system filesystem REQUIRED)


# DEFINITIONS
ADD_DEFINITIONS(${OpenCV_DEFINITIONS})

# INCLUDES
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

# LIB DIRS
LINK_DIRECTORIES(${Boost_LIBRARY_DIR})

# LIBRARY
SET(
    WAVE_VISION_DEPS
    yaml-cpp
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    wave_kinematics
    wave_utils
)
ADD_LIBRARY(
    wave_vision
    STATIC
    src/utils.cpp
    src/dataset.cpp
    src/detector/fast_detector.cpp
    src/descriptor/brisk_descriptor.cpp
)
TARGET_LINK_LIBRARIES(wave_vision ${WAVE_VISION_DEPS})

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests
              tests/descriptor_tests/brisk_tests.cpp
              tests/detector_tests/fast_tests.cpp
              tests/dataset_tests.cpp
              tests/utils_tests.cpp)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${Boost_LIBRARIES}
    ${PROJECT_NAME}
    wave_kinematics
    wave_utils
)
# COPY TEST DATA
FILE(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
