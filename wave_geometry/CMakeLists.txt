PROJECT(wave_geometry)

# EXTERNAL PACKAGES
FIND_PACKAGE(kindr REQUIRED)

ADD_LIBRARY(${PROJECT_NAME} STATIC
    src/rotation.cpp)

# Use these headers when building, and make clients use them
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    wave_utils
    Eigen3::Eigen
    kindr)

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests tests/rotation_test.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
