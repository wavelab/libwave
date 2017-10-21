PROJECT(wave_geometry)

# EXTERNAL PACKAGES
FIND_PACKAGE(kindr REQUIRED)

# Define an imported target for kindr
ADD_LIBRARY(Kindr INTERFACE IMPORTED)
SET_PROPERTY(TARGET Kindr PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "${kindr_INCLUDE_DIRS}")


WAVE_ADD_LIBRARY(${PROJECT_NAME}
    src/rotation.cpp)

# Use these headers when building, and make clients use them
WAVE_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    Wave::utils
    Eigen3::Eigen
    Kindr)

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests tests/rotation_test.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
