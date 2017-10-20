PROJECT(wave_controls)

ADD_LIBRARY(${PROJECT_NAME}
    src/pid.cpp)

# Use these headers when building, and make clients use them
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen)

# Set install destinations for all related files
WAVE_INSTALL(${PROJECT_NAME})

# UNIT TESTS
WAVE_ADD_TEST(${PROJECT_NAME}_tests tests/pid_test.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
