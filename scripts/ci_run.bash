#!/bin/bash
set -e  # exit on first error

compile_libwave() {
    mkdir -p build
    cd build
    cmake ..
    make
}

run_module_tests() {
    cd $1 && ./tests/$1_tests --silence-stdcout && cd -
}

# MAIN
compile_libwave
run_module_tests wave_utils
run_module_tests wave_optimization
