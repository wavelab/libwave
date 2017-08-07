CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_optimization)

# PACKAGES
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR})

# LIBRARY
SET(WAVE_OPTIMIZATION_DEPS
    ${OpenCV_LIBS}
    wave_kinematics
    wave_utils
)
ADD_LIBRARY(
    wave_optimization
    STATIC
    src/ceres/ba.cpp
    src/ceres/ceres_examples.cpp
)
TARGET_LINK_LIBRARIES(wave_optimization ${WAVE_OPTIMIZATION_DEPS})

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests
             tests/ceres/ba_test.cpp
             tests/ceres/ceres_examples_test.cpp)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_vision
    wave_utils
    ${CERES_LIBRARIES}
)

# COPY TEST DATA
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
