#!/bin/bash
set -e  # exit on first error
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"
CMAKE_CONFIG_DIR="../cmake"  # cmake folder relative to this script

compile_libwave() {
    mkdir -p build
    cd build
    cmake .. -DCMAKE_MODULE_PATH=$DIR/$CMAKE_CONFIG_DIR
    make
}

run_module_tests() {
    cd $1 && ./tests/$1_tests --silence-stdcout && cd -
}

# MAIN
compile_libwave
run_module_tests wave_controls
run_module_tests wave_geometry
run_module_tests wave_kinematics
run_module_tests wave_optimization
# run_module_tests wave_matching
run_module_tests wave_utils
