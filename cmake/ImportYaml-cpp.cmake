# - Fixes imported target for yaml-cpp
# For some reason yaml-cpp does not have include directories in its imported
# target. Add them to the target, if not already set.
IF(TARGET yaml-cpp)
    GET_TARGET_PROPERTY(current_property yaml-cpp INTERFACE_INCLUDE_DIRECTORIES)
    IF(NOT current_property)
        SET_TARGET_PROPERTIES(yaml-cpp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
            "${YAML_CPP_INCLUDE_DIR}")
    ENDIF()
ENDIF()
