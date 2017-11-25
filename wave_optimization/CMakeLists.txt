PROJECT(wave_optimization)

WAVE_ADD_MODULE(${PROJECT_NAME} DEPENDS
    wave::utils
    wave::kinematics
    wave::vision
    Eigen3::Eigen
    ceres
    gtsam
    SOURCES
    src/ceres/ba.cpp
    src/ceres/ceres_examples.cpp
    src/gtsam/hand_eye.cpp)

# Unit tests
IF(BUILD_TESTING)
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
            wave::vision
            ${GTSAM_LIBRARIES})

        WAVE_ADD_TEST(vo_gtsam_offline_kitti_example
            tests/gtsam/gtsam_offline_kitti_example.cpp)
        TARGET_LINK_LIBRARIES(vo_gtsam_offline_kitti_example
            ${PROJECT_NAME}
            wave::vision
            ${GTSAM_LIBRARIES})

        WAVE_ADD_TEST(hand_eye_gtsam_test
                tests/gtsam/hand_eye_test.cpp)
        TARGET_LINK_LIBRARIES(hand_eye_gtsam_test
                ${PROJECT_NAME}
                ${GTSAM_LIBRARIES})
    ENDIF()

    # COPY TEST DATA
    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
ENDIF(BUILD_TESTING)