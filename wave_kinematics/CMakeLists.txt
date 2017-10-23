PROJECT(wave_kinematics)

ADD_LIBRARY(${PROJECT_NAME} STATIC
    src/gimbal.cpp
    src/pose.cpp
    src/quadrotor.cpp
    src/two_wheel.cpp)

# Use these headers when building, and make clients use them
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    wave_utils
    wave_controls
    Eigen3::Eigen)

# UNIT TESTS
WAVE_ADD_TEST(
    ${PROJECT_NAME}_tests
    tests/gimbal_test.cpp
    tests/pose_test.cpp
    tests/quadrotor_test.cpp
    tests/two_wheel_test.cpp
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
