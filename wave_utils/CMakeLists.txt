PROJECT(wave_utils)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    Eigen3::Eigen
    yaml-cpp
    opencv_core
    SOURCES
    src/config.cpp
    src/data.cpp
    src/file.cpp
    src/math.cpp
    src/time.cpp)

# Unit tests
WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/utils/config_test.cpp
    tests/utils/data_test.cpp
    tests/utils/file_test.cpp
    tests/utils/math_test.cpp
    tests/utils/time_test.cpp
)
TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
)

# COPY TEST DATA
FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
