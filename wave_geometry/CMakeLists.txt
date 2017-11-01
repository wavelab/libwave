PROJECT(wave_geometry)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::utils
    Eigen3::Eigen
    Kindr::kindr
    SOURCES
    src/rotation.cpp)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests tests/rotation_test.cpp)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
ENDIF(BUILD_TESTING)
