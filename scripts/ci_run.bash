#!/bin/bash
set -e  # exit on first error

# compile libwave
mkdir -p build
cd build
cmake ..
make

# run tests
cd wave_utils && ./tests/wave_utils_tests --silence-stdcout && cd -
