PROJECT(wave_geometry)

WAVE_ADD_LIBRARY(${PROJECT_NAME}
    src/rotation.cpp)

# Use these headers when building, and make clients use them
WAVE_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    wave::utils
    Eigen3::Eigen
    Kindr::kindr)

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests tests/rotation_test.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
