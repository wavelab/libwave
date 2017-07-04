CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_vision)
# SET(CMAKE_BUILD_TYPE RELEASE)

# DEPENDENCIES
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system filesystem REQUIRED)

# DEFINITIONS
ADD_DEFINITIONS(${OpenCV_DEFINITIONS})

# INCLUDES
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

# LIBRARY
SET(
    WAVE_VISION_DEPS
    yaml-cpp
    ${OpenCV_LIBS}
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
    src/matcher/brute_force_matcher.cpp
)
TARGET_LINK_LIBRARIES(wave_vision ${WAVE_VISION_DEPS})

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests
              tests/detector_tests/fast_tests.cpp
              tests/descriptor_tests/brisk_tests.cpp
              tests/matcher_tests/brute_force_tests.cpp
              tests/tracker_tests/tracker_tests.cpp
              tests/dataset_tests.cpp
              tests/utils_tests.cpp)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_kinematics
    wave_utils
    ${Boost_LIBRARIES}
)
# COPY TEST DATA
FILE(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
