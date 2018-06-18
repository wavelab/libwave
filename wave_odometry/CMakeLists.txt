PROJECT(wave_odometry)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::optimization
    wave::geometry
    wave::kinematics
    wave::matching
    PCL::PCL
    libnabo::libnabo
    SOURCES
    src/weighting_functions.cpp
    src/integrals.cpp
    src/sensor_model.cpp
    src/transformer.cpp
    src/feature_extractor.cpp
    src/LaserOdom.cpp)

IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
            tests/laserodom_tests.cpp)
    TARGET_LINK_LIBRARIES(
            ${PROJECT_NAME}_tests
            ${PROJECT_NAME})

    WAVE_ADD_TEST(feature_extraction_tests
            tests/feature_extractor_tests.cpp)
    TARGET_LINK_LIBRARIES(feature_extraction_tests
            ${PROJECT_NAME})

    WAVE_ADD_TEST(transformer_tests
            tests/transformer_tests.cpp)
    TARGET_LINK_LIBRARIES(transformer_tests
            ${PROJECT_NAME})

    WAVE_ADD_TEST(residual_tests
            tests/residual_test/implicit_line_test.cpp)
    TARGET_LINK_LIBRARIES(residual_tests
            ${PROJECT_NAME})

    WAVE_ADD_BENCHMARK(feature_benchmark
            tests/feature_extractor_benchmark.cpp)
    TARGET_LINK_LIBRARIES(feature_benchmark
            ${PROJECT_NAME})

    WAVE_ADD_BENCHMARK(transformer_benchmark
            tests/transformer_benchmark.cpp)
    TARGET_LINK_LIBRARIES(transformer_benchmark
            ${PROJECT_NAME})

    WAVE_ADD_BENCHMARK(nn_benchmark
            tests/nn_benchmark.cpp)
    TARGET_LINK_LIBRARIES(nn_benchmark
            ${PROJECT_NAME})

    # Copy the test data
    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
    FILE(COPY tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)

    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/benchmarks)
    FILE(COPY tests/config DESTINATION ${PROJECT_BINARY_DIR}/benchmarks)
ENDIF(BUILD_TESTING)
