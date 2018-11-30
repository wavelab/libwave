PROJECT(wave_geometry_og)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::utils
    Eigen3::Eigen
    Kindr::kindr)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
            tests/transformation_test.cpp
            tests/interpolated_transform_test.cpp)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests
            ${PROJECT_NAME}
            wave::kinematics)
ENDIF(BUILD_TESTING)
