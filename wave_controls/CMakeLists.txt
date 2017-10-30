PROJECT(wave_controls)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    Eigen3::Eigen
    SOURCES
    src/pid.cpp)

# Unit tests
WAVE_ADD_TEST(${PROJECT_NAME}_tests tests/pid_test.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
