#!/bin/bash

# This variable can be set to use a different version of clang-format
: ${FORMAT_EXE:=clang-format}

# Format all source files. Run from the source root directory.
find ./wave_*/ -name "*.hpp" -o -name "*.cpp" | xargs ${FORMAT_EXE} -i
