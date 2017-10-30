PROJECT(wave_benchmark)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::utils
    wave::geometry
    wave::containers
    Eigen3::Eigen
    SOURCES
    src/trajectory_compare.cpp)

# Unit tests
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/benchmark/benchmark_tests.cpp
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
