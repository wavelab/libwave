PROJECT(wave_odometry)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::optimization
    wave::geometry
    wave::kinematics
    wave::utils
    libnabo::libnabo
    PCL::PCL
    SOURCES
    src/rigid_residual.cpp
    src/weighting_functions.cpp
    src/odometry_callback.cpp
    src/integrals.cpp
    src/sensor_model.cpp
    src/transformer.cpp
    src/line_fitter.cpp
    src/icosahedron_binner.cpp
    src/feature_extractor.cpp
    src/laser_odom.cpp)

ADD_LIBRARY(kitti_utils
        kitti_tools/src/kitti_utility_methods.cpp)
TARGET_LINK_LIBRARIES(kitti_utils
        wave::geography
        wave::matching
        ${PROJECT_NAME})

ADD_LIBRARY(config_utils
        tests/data/src/config_utils.cpp)
TARGET_LINK_LIBRARIES(config_utils
        wave::matching
        ${PROJECT_NAME})

ADD_EXECUTABLE(kitti_raw_eval
        kitti_tools/src/kitti_raw_evaluator.cpp)
TARGET_LINK_LIBRARIES(kitti_raw_eval
        kitti_utils
        config_utils)

ADD_EXECUTABLE(kitti_odom_eval
        kitti_tools/src/kitti_odom_evaluator.cpp)
TARGET_LINK_LIBRARIES(kitti_odom_eval
        kitti_utils
        config_utils)

ADD_EXECUTABLE(moose_eval
        kitti_tools/src/moose_evaluator.cpp)
TARGET_LINK_LIBRARIES(moose_eval
        kitti_utils
        config_utils)

IF(BUILD_TESTING)
    WAVE_ADD_TEST(feature_extraction_tests
            tests/feature_extractor/feature_extractor_tests.cpp)
    TARGET_LINK_LIBRARIES(feature_extraction_tests
            ${PROJECT_NAME}
            wave::matching)

    WAVE_ADD_BENCHMARK(feature_benchmark
            tests/feature_extractor/feature_extractor_benchmark.cpp)
    TARGET_LINK_LIBRARIES(feature_benchmark
            ${PROJECT_NAME})


    WAVE_ADD_TEST(transformer_tests
            tests/transformer/transformer_tests.cpp)
    TARGET_LINK_LIBRARIES(transformer_tests
            ${PROJECT_NAME}
            wave::matching)

    WAVE_ADD_BENCHMARK(transformer_benchmark
            tests/transformer/transformer_benchmark.cpp)
    TARGET_LINK_LIBRARIES(transformer_benchmark
            ${PROJECT_NAME}
            wave::matching)


    WAVE_ADD_TEST(residual_tests
            tests/residual_test/residual_test.cpp)
    TARGET_LINK_LIBRARIES(residual_tests
            ${PROJECT_NAME})

    WAVE_ADD_BENCHMARK(residual_benchmark
            tests/residual_test/residual_benchmark.cpp)
    TARGET_LINK_LIBRARIES(residual_benchmark
            ${PROJECT_NAME})

    WAVE_ADD_TEST(callback_tests
            tests/callback/odometry_callback_test.cpp)
    TARGET_LINK_LIBRARIES(callback_tests
            ${PROJECT_NAME})

    WAVE_ADD_TEST(binner_tests
            tests/icosahedron_binner_test.cpp)
    TARGET_LINK_LIBRARIES(binner_tests
            ${PROJECT_NAME})

    WAVE_ADD_TEST(line_fitter_tests
            tests/line_fitter/line_fitter_test.cpp)
    TARGET_LINK_LIBRARIES(line_fitter_tests
            ${PROJECT_NAME}
            config_utils
            kitti_utils)

    WAVE_ADD_BENCHMARK(nn_benchmark
            tests/knn/nn_benchmark.cpp)
    TARGET_LINK_LIBRARIES(nn_benchmark
            ${PROJECT_NAME})

    # Copy the test data
    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
    FILE(COPY tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)

    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/benchmarks)
    FILE(COPY tests/config DESTINATION ${PROJECT_BINARY_DIR}/benchmarks)
ENDIF(BUILD_TESTING)
