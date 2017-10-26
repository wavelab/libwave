PROJECT(wave_kinematics)

WAVE_ADD_LIBRARY(${PROJECT_NAME}
    src/gimbal.cpp
    src/pose.cpp
    src/quadrotor.cpp
    src/two_wheel.cpp)

# Use these headers when building, and make clients use them
WAVE_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC include)

# Depend on these modules and external libraries, and make clients use them
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    wave::utils
    wave::controls
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
