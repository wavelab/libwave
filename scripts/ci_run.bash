#!/bin/bash
set -e  # exit on first error
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"
CMAKE_CONFIG_DIR="../cmake"  # cmake folder relative to this script

compile_libwave() {
    mkdir -p build
    cd build
    export Eigen3_DIR="/usr/include/eigen3:$Eigen3_DIR"
    cmake .. -DCMAKE_MODULE_PATH=$DIR/$CMAKE_CONFIG_DIR
    make
}

run_module_tests() {
    cd $1 && ./tests/$1_tests --silence-stdcout && cd -
}

# MAIN
compile_libwave
run_module_tests wave_utils
run_module_tests wave_optimization
