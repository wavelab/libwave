CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wave)


INCLUDE_DIRECTORIES(
    wave_estimation/include
    wave_gimbal/include
    wave_kinematics/include
    wave_optimization/include
    wave_symmath/include
    wave_vision/include
    wave_utils/include
)


# libwave_estimation.a
ADD_SUBDIRECTORY(wave_estimation)
ADD_DEPENDENCIES(wave_estimation wave_utils wave_kinematics)

# libwave_gimbal.a
ADD_SUBDIRECTORY(wave_gimbal)
ADD_DEPENDENCIES(wave_gimbal wave_utils)

# libwave_kinematics.a
ADD_SUBDIRECTORY(wave_kinematics)
ADD_DEPENDENCIES(wave_kinematics wave_utils)

# libwave_optimization.a
ADD_SUBDIRECTORY(wave_optimization)
ADD_DEPENDENCIES(wave_optimization wave_utils)

# libwave_symath.a
ADD_SUBDIRECTORY(wave_symmath)
ADD_DEPENDENCIES(wave_symmath wave_utils)

# libwave_utils.a
ADD_SUBDIRECTORY(wave_utils)

# libwave_vision.a
ADD_SUBDIRECTORY(wave_vision)
ADD_DEPENDENCIES(wave_vision wave_utils)

# libwave_viz.a
ADD_SUBDIRECTORY(wave_viz)
ADD_DEPENDENCIES(wave_viz wave_utils)


# libwave.a
ADD_CUSTOM_TARGET(
    libwave ALL
    COMMAND
        ${CMAKE_AR} rc libwave.a
            ${CMAKE_BINARY_DIR}/wave_estimation/libwave_estimation.a
            ${CMAKE_BINARY_DIR}/wave_gimbal/libwave_gimbal.a
            ${CMAKE_BINARY_DIR}/wave_kinematics/libwave_kinematics.a
            ${CMAKE_BINARY_DIR}/wave_optimization/libwave_optimization.a
            ${CMAKE_BINARY_DIR}/wave_symmath/libwave_symmath.a
            ${CMAKE_BINARY_DIR}/wave_utils/libwave_utils.a
            ${CMAKE_BINARY_DIR}/wave_vision/libwave_vision.a
    DEPENDS
        wave_estimation
        wave_gimbal
        wave_kinematics
        wave_optimization
        wave_symmath
        wave_vision
        wave_utils
)
