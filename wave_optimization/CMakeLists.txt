CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_optimization)

# PACKAGES
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

# LIBRARY
ADD_LIBRARY(wave_optimization
            STATIC
            src/ceres/ceres_examples.cpp)

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests
              tests/ceres/ceres_examples_test.cpp
              tests/factor_graph/factor_test.cpp
              tests/factor_graph/graph_test.cpp
              tests/factor_graph/variable_test.cpp
              tests/factor_graph/noise_test.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests
                      ${PROJECT_NAME}
                      wave_utils
                      ${CERES_LIBRARIES})

WAVE_ADD_TEST(${PROJECT_NAME}_examples
    tests/factor_graph/graph_slam_2d.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_examples
    ${PROJECT_NAME}
    wave_kinematics
    ${CERES_LIBRARIES})
