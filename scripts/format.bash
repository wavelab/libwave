#!/bin/bash

# Usage: format.bash [path to search]
# clang-format will be called for each .hpp and .cpp file in the given path(s)
# The default is: ./wave_*/

SEARCH_PATHS=${@:-./wave_*/}

# This variable can be set to use a different version of clang-format
: ${FORMAT_EXE:=clang-format}

# Format all source files. Run from the source root directory.
find ${SEARCH_PATHS} -name "*.hpp" -o -name "*.cpp" | xargs ${FORMAT_EXE} -i
