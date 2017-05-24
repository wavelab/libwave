#!/bin/bash
set -e  # exit on first error
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"
CMAKE_CONFIG_DIR="../cmake"  # cmake folder relative to this script

compile_libwave() {
    mkdir -p build
    cd build
    cmake .. -DCMAKE_MODULE_PATH=$DIR/$CMAKE_CONFIG_DIR
    make -j$(nproc)
}

test_libwave() {
    export GTEST_COLOR=1
    ctest --output-on-failure
}

# MAIN
compile_libwave
test_libwave
