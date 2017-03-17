
CERES_CONFIG = {
    occupied_space_weight_0 = 10.,
    translation_weight = 0.01,
    rotation_weight = 0.1,
    only_optimize_yaw = false,
    ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 100,
        num_threads = 7,
    },
}

return CERES_CONFIG