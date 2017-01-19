CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(wavelib)


INCLUDE_DIRECTORIES(
    wavelib_estimation/include
    wavelib_gimbal/include
    wavelib_kinematics/include
    wavelib_optimization/include
    wavelib_symmath/include
    wavelib_vision/include
    wavelib_utils/include
)


# libwavelib_estimation.a
ADD_SUBDIRECTORY(wavelib_estimation)
ADD_DEPENDENCIES(wavelib_estimation wavelib_utils wavelib_kinematics)

# libwavelib_gimbal.a
ADD_SUBDIRECTORY(wavelib_gimbal)
ADD_DEPENDENCIES(wavelib_gimbal wavelib_utils)

# libwavelib_kinematics.a
ADD_SUBDIRECTORY(wavelib_kinematics)
ADD_DEPENDENCIES(wavelib_kinematics wavelib_utils)

# libwavelib_optimization.a
ADD_SUBDIRECTORY(wavelib_optimization)
ADD_DEPENDENCIES(wavelib_optimization wavelib_utils)

# libwavelib_symath.a
ADD_SUBDIRECTORY(wavelib_symmath)
ADD_DEPENDENCIES(wavelib_symmath wavelib_utils)

# libwavelib_utils.a
ADD_SUBDIRECTORY(wavelib_utils)

# libwavelib_vision.a
ADD_SUBDIRECTORY(wavelib_vision)
ADD_DEPENDENCIES(wavelib_vision wavelib_utils)

# libwavelib_viz.a
ADD_SUBDIRECTORY(wavelib_viz)
ADD_DEPENDENCIES(wavelib_viz wavelib_utils)


# libwavelib.a
ADD_CUSTOM_TARGET(
    libwavelib ALL
    COMMAND
        ${CMAKE_AR} rc libwavelib.a
            ${CMAKE_BINARY_DIR}/wavelib_estimation/libwavelib_estimation.a
            ${CMAKE_BINARY_DIR}/wavelib_gimbal/libwavelib_gimbal.a
            ${CMAKE_BINARY_DIR}/wavelib_kinematics/libwavelib_kinematics.a
            ${CMAKE_BINARY_DIR}/wavelib_optimization/libwavelib_optimization.a
            ${CMAKE_BINARY_DIR}/wavelib_symmath/libwavelib_symmath.a
            ${CMAKE_BINARY_DIR}/wavelib_utils/libwavelib_utils.a
            ${CMAKE_BINARY_DIR}/wavelib_vision/libwavelib_vision.a
    DEPENDS
        wavelib_estimation
        wavelib_gimbal
        wavelib_kinematics
        wavelib_optimization
        wavelib_symmath
        wavelib_vision
        wavelib_utils
)
