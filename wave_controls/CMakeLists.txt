PROJECT(wave_controls)

ADD_LIBRARY(${PROJECT_NAME} STATIC
    src/pid.cpp)

# Use these headers when building, and make clients use them
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests tests/pid_test.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
