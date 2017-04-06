# wave/utils/logging.hpp

This module contains functions to log errors and info to `stderr` and `stdout`.


## Macros

    namespace wave {

    #define LOG_ERROR(M, ...)
    #define LOG_INFO(M, ...)

    }  // end of wave namespace

`LOG_ERROR` and `LOG_INFO` both are simple `fprintf()` that can be use to write message `M` to `stderr` and `stdout`. For example:

    LOG_ERROR("Failed to load configuration file [%s]", config_file.c_str());
    LOG_INFO("Parameter was not found! Loading defaults!");
