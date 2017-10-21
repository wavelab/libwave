PROJECT(wave_benchmark)

ADD_LIBRARY(${PROJECT_NAME}
    src/trajectory_compare.cpp)

# Use these headers when building, and make clients use them
WAVE_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    wave_utils
    wave_geometry
    wave_containers
    Eigen3::Eigen)

# Set install destinations for all related files
WAVE_INSTALL(${PROJECT_NAME})

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/benchmark/benchmark_tests.cpp
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
