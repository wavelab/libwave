PROJECT(wave_optimization)

# PACKAGES
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(GTSAM) # optional for now; used only in tests (@todo)

ADD_LIBRARY(${PROJECT_NAME} STATIC
    src/ceres/ba.cpp
    src/ceres/ceres_examples.cpp)

# Use these headers when building, and make clients use them
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC SYSTEM ${CERES_INCLUDE_DIRS})

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    wave_utils
    wave_kinematics
    wave_vision
    Eigen3::Eigen
    ${CERES_LIBRARIES})

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests
             tests/ceres/ba_test.cpp
             tests/ceres/ceres_examples_test.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})

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
