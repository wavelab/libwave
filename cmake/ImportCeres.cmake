# - Fixes imported target for ceres-solver
# For some reason ceres does not have include directories in its imported
# target. Add them to the target, if not already set.
IF(TARGET ceres)
    GET_TARGET_PROPERTY(current_property ceres INTERFACE_INCLUDE_DIRECTORIES)
    IF(NOT current_property)
        SET_TARGET_PROPERTIES(ceres PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
            "${CERES_INCLUDE_DIRS}")
    ENDIF()
ENDIF()
