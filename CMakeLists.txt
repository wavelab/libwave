CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(slam)


INCLUDE_DIRECTORIES(
    slam_estimation/include
    slam_gimbal/include
    slam_kinematics/include
    slam_optimization/include
    slam_symmath/include
    slam_vision/include
    slam_utils/include
)


# libslam_estimation.a
ADD_SUBDIRECTORY(slam_estimation)
ADD_DEPENDENCIES(slam_estimation slam_utils slam_kinematics)

# libslam_gimbal.a
ADD_SUBDIRECTORY(slam_gimbal)
ADD_DEPENDENCIES(slam_gimbal slam_utils)

# libslam_kinematics.a
ADD_SUBDIRECTORY(slam_kinematics)
ADD_DEPENDENCIES(slam_kinematics slam_utils)

# libslam_optimization.a
ADD_SUBDIRECTORY(slam_optimization)
ADD_DEPENDENCIES(slam_optimization slam_utils)

# libslam_symath.a
ADD_SUBDIRECTORY(slam_symmath)
ADD_DEPENDENCIES(slam_symmath slam_utils)

# libslam_utils.a
ADD_SUBDIRECTORY(slam_utils)

# libslam_vision.a
ADD_SUBDIRECTORY(slam_vision)
ADD_DEPENDENCIES(slam_vision slam_utils)

# libslam_viz.a
ADD_SUBDIRECTORY(slam_viz)
ADD_DEPENDENCIES(slam_viz slam_utils)


# libslam.a
ADD_CUSTOM_TARGET(
    libslam ALL
    COMMAND
        ${CMAKE_AR} rc libslam.a
            ${CMAKE_BINARY_DIR}/slam_estimation/libslam_estimation.a
            ${CMAKE_BINARY_DIR}/slam_gimbal/libslam_gimbal.a
            ${CMAKE_BINARY_DIR}/slam_kinematics/libslam_kinematics.a
            ${CMAKE_BINARY_DIR}/slam_optimization/libslam_optimization.a
            ${CMAKE_BINARY_DIR}/slam_symmath/libslam_symmath.a
            ${CMAKE_BINARY_DIR}/slam_utils/libslam_utils.a
            ${CMAKE_BINARY_DIR}/slam_vision/libslam_vision.a
    DEPENDS
        slam_estimation
        slam_gimbal
        slam_kinematics
        slam_optimization
        slam_symmath
        slam_vision
        slam_utils
)
