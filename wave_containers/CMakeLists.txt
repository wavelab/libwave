PROJECT(wave_containers)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    Eigen3::Eigen
    Boost::boost)

# Unit tests
WAVE_ADD_TEST(${PROJECT_NAME}_tests
    tests/measurement_test.cpp
    tests/landmark_measurement_test.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})

IF(BUILD_BENCHMARKS)
    WAVE_ADD_BENCHMARK(${PROJECT_NAME}_benchmark
        tests/measurement_container_benchmark.cpp)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_benchmark ${PROJECT_NAME})
ENDIF(BUILD_BENCHMARKS)