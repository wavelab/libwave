PROJECT(wave_odometry)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::optimization
    wave::geometry
    wave::kinematics
    wave::matching
    PCL::PCL
    SOURCES
    src/weighting_functions.cpp
    src/integrals.cpp
    src/sensor_model.cpp
    src/feature_extractor.cpp
    src/LaserOdom.cpp)

IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
            tests/laserodom_tests.cpp)
    TARGET_LINK_LIBRARIES(
        ${PROJECT_NAME}_tests
        ${PROJECT_NAME})

    # Copy the test data
    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
    FILE(COPY tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
ENDIF(BUILD_TESTING)
