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
- OpenCV 3.2.0
- PCL 1.8
- yaml-cpp 0.5.1
- CMake 2.8.3
- GCC 5.4
- gtsam
- GeographicLib 1.49

The above versions are the minimum we test against.
Some earlier versions may work, but are not tested.
For convenience, we provide a script `scripts/install/install_deps.bash`,
which installs these dependencies (except GCC, GeographicLib, gtsam,
and OpenCV) on Ubuntu 14.04 or Ubuntu 16.04.

The following scripts are available for installing GeographicLib, gtsam, and OpenCV:
- `scripts/install/install_geographiclib.bash`
- `scripts/install/install_gtsam.bash`
- `scripts/install/install_opencv.bash`

The dependencies for each module are:

| Module | Dependencies |
| ------ | ------------ |
| wave\_benchmark | <ul><li>Eigen3</li><li>wave\_containers</li><li>wave\_geometry</li><li>wave\_utils</li></ul> |
| wave\_containers | <ul><li>Boost</li><li>Eigen3</li></ul> |
| wave\_controls | <ul><li>Eigen3</li></ul> |
| wave\_geography | <ul><li>Eigen3</li><li>GeographicLib</li></ul> |
| wave\_geometry | <ul>[See dependencies here](https://github.com/wavelab/wave_geometry/tree/27be85b37f25b8bb7ec4cd3fc189d3fc9fb7361d#dependencies)</ul> |
| wave\_gtsam | <ul><li>Eigen3</li><li>gtsam</li><li>wave\_utils</li></ul> |
| wave\_kinematics | <ul><li>Eigen3</li><li>wave\_controls</li><li>wave\_utils</li></ul> |
| wave\_matching | <ul><li>Boost</li><li>Eigen3</li><li>PCL</li><li>wave\_utils</li></ul> |
| wave\_optimization | <ul><li>ceres</li><li>Eigen3</li><li>wave\_kinematics</li><li>wave\_utils</li><li>wave\_vision</li></ul> |
| wave\_utils | <ul><li>Eigen3</li><li>yaml-cpp</li></ul> |
| wave\_vision | <ul><li>Boost Filesystem</li><li>Eigen3</li><li>opencv\_calib3d</li><li>opencv\_core</li><li>opencv\_features2d</li><li>opencv\_highgui</li><li>opencv\_imgproc</li><li>opencv\_videoio</li><li>wave\_containers</li><li>wave\_kinematics</li><li>wave\_utils</li></ul> |

## Install

To build libwave from source (and install some dependencies) run the following
script in the terminal, in the directory you wish to contain the `libwave`
source directory:

    curl -L https://git.io/vyKXR > install.bash && bash install.bash

The above script performs the following tasks:

    # clone repo with submodules
    git clone --recursive https://github.com/wavelab/libwave.git
    cd libwave

    # install dependencies
    bash scripts/install/install_deps.bash

    # compile libwave
    mkdir -p build
    cd build
    cmake ..
    make -j8

Install libwave with `make install`. Alternatively, you can enable the
`EXPORT_BUILD` option in CMake, which will make the libwave build directory 
searchable by CMake without installation.


**Note**: dependency install scripts are provided for convenience and are not
tested on a wide variety of systems. They may be incompatible with other
packages installed on your system. Additional scripts which install
GeographicLib, gtsam, and OpenCV 3 are provided in `scripts/install`.

**Known issue with OpenCV**: `wave_vision` may
[fail to build](https://github.com/wavelab/libwave/issues/267) if OpenCV 2 is
installed in the default system directory on Ubuntu 16.04 (and OpenCV 3 is
installed elsewhere). The recommended solution is to remove OpenCV 2 via
`sudo apt remove libopencv-dev`.


## Use with CMake

One libwave has been either installed or exported by CMake, it can be used in
your project's `CMakeLists.txt` file as follows:

    cmake_minimum_required(VERSION 3.0)
    project(example)

    find_package(wave REQUIRED geometry matching)

    add_executable(example example.cpp)
    target_link_libraries(example wave::geometry wave::matching)


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

Copyright (c) 2017 Waterloo Autonomous Vehicles Laboratory (WAVELab)

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
