# libwave
[![Build Status](https://travis-ci.org/wavelab/libwave.png?branch=master)][1]

This library contains reusable code for:

- Estimation
- Kinematics
- Optimization
- Vision
- and more ..

## Requirements

- Boost 1.54
- Ceres 1.13
- Eigen 3.3.2
- Kindr 1.0.4
- OpenCV 3.2.0
- PCL 1.8
- yaml-cpp 0.5.1
- CMake 2.8.3
- GCC 5.4

The above versions are the minimum we test against.
Some earlier versions may work, but are not tested.
For convenience, we provide a script which installs these dependencies (except 
GCC) on Ubuntu 14.04 or Ubuntu 16.04, in `scripts/install/install_deps.bash`.

## Install

Execute the following in the terminal where you want libwave to reside:

    curl -L https://git.io/vyKXR > install.bash && bash install.bash

**Or** you can perform the installation manually:

    # clone repo
    git clone https://github.com/wavelab/libwave.git
    cd libwave

    # initialize git submodules
    git submodule init
    git submodule update

    # install dependencies
    bash scripts/install/install_deps.bash

    # compile libwave
    mkdir -p build
    cd build
    cmake ..
    make


## Documentation

The documentation consists of an API reference as well as pages on Fundamentals,
describing concepts and theory behind the library.

It is generated using Doxygen, using the build target `make doc`.


## Notes for Developers

To expedite our development process we use a set of git hooks to run:

- Clang format (pre-commit)

these can be activated by running the `githook_init.bash` from the root of the repo

    bash scripts/githooks/githooks_init.bash


## LICENSE

Copyright (c) <2017> <Wavelab>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

[1]: https://travis-ci.org/wavelab/libwave
[edit_docs]: http://chutsu.github.io/ditto/#docs/how_do_i_use_ditto
