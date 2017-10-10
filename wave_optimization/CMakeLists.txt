CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave_optimization)

# PACKAGES
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(GTSAM)

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
    src/ceres/SE3Parameterization.cpp
    src/ceres/point_to_point_residual.cpp
)
TARGET_LINK_LIBRARIES(wave_optimization ${WAVE_OPTIMIZATION_DEPS})

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests
             tests/ceres/ba_test.cpp
             tests/ceres/ceres_examples_test.cpp
            tests/ceres/SE3_local_parameter_test.cpp)

TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_vision
    wave_utils
    wave_geometry
    ${CERES_LIBRARIES}
)

IF(GTSAM_FOUND)
    # Set for consistency, since GTSAM's config file does not
    SET(GTSAM_LIBRARIES gtsam)
    INCLUDE_DIRECTORIES(${GTSAM_INCLUDE_DIR})

    WAVE_ADD_TEST(vo_gtsam_offline_example
        tests/gtsam/gtsam_offline_example.cpp)
    TARGET_LINK_LIBRARIES(vo_gtsam_offline_example
        ${PROJECT_NAME}
        wave_vision
        ${GTSAM_LIBRARIES})

    WAVE_ADD_TEST(vo_gtsam_offline_kitti_example
        tests/gtsam/gtsam_offline_kitti_example.cpp)
    TARGET_LINK_LIBRARIES(vo_gtsam_offline_kitti_example
        ${PROJECT_NAME}
        wave_vision
        ${GTSAM_LIBRARIES})
ENDIF()

# COPY TEST DATA
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
