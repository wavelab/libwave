PROJECT(wave_gtsam)
# Extensions for GTSAM library, such as combined motion factors

WAVE_ADD_MODULE(${PROJECT_NAME} DEPENDS
    wave::utils
    Eigen3::Eigen
    gtsam
    SOURCES
    src/hand_eye.cpp
    src/decaying_bias.cpp
    src/gps_factor_with_bias.cpp
    src/pose_vel.cpp
    src/pose_vel_bias.cpp
    src/pose_vel_imubias.cpp
    src/preint_imu_factor.cpp)

# Unit tests
IF(BUILD_TESTING)

    # These tests also use wave_vision
    IF(TARGET wave::vision)
        WAVE_ADD_TEST(wave_gtsam_vo_offline_example
            tests/gtsam/gtsam_offline_example.cpp)
        TARGET_LINK_LIBRARIES(wave_gtsam_vo_offline_example
            ${PROJECT_NAME}
            wave::vision)


        WAVE_ADD_TEST(wave_gtsam_vo_offline_kitti_example
            tests/gtsam/gtsam_offline_kitti_example.cpp)
        TARGET_LINK_LIBRARIES(wave_gtsam_vo_offline_kitti_example
            ${PROJECT_NAME}
            wave::vision)

        # Copy the test data stored in wave_optimization
        FILE(COPY ../wave_optimization/tests/data
            DESTINATION ${PROJECT_BINARY_DIR}/tests)
    ENDIF()


    WAVE_ADD_TEST(wave_gtsam_hand_eye_test
            tests/gtsam/hand_eye_test.cpp
            tests/gtsam/gps_factor_with_bias_test.cpp)
    TARGET_LINK_LIBRARIES(wave_gtsam_hand_eye_test
            ${PROJECT_NAME})

    WAVE_ADD_TEST(wave_gtsam_decaying_bias_test
            tests/gtsam/decaying_bias_test.cpp)
    TARGET_LINK_LIBRARIES(wave_gtsam_decaying_bias_test
            ${PROJECT_NAME})

    WAVE_ADD_TEST(wave_gtsam_pose_vel_state_test
            tests/gtsam/pose_vel_state_test.cpp)
    TARGET_LINK_LIBRARIES(wave_gtsam_pose_vel_state_test
            ${PROJECT_NAME})

    WAVE_ADD_TEST(wave_gtsam_prior_test
            tests/gtsam/prior_tests.cpp)
    TARGET_LINK_LIBRARIES(wave_gtsam_prior_test
            ${PROJECT_NAME})

    WAVE_ADD_TEST(wave_gtsam_imu_preint_test
        tests/gtsam/imu_preint_test.cpp)
    TARGET_LINK_LIBRARIES(wave_gtsam_imu_preint_test
        ${PROJECT_NAME})
ENDIF(BUILD_TESTING)
