#libwave
[![Build Status](https://travis-ci.org/wavelab/libwave.png?branch=master)][1]

This library contains reusable code for:

- Estimation
- Kinematics
- Optimization
- Vision
- and more ..


## Install

Execute the following in the terminal where you want libwave to reside:

    curl -L https://git.io/vyKXR > install.bash && bash install.bash

**Or** you can perform the installation manually:

#clone repo
    git clone https://github.com/wavelab/libwave.git
    cd libwave

#initialize git submodules
    git submodule init
    git submodule update

#install dependencies
    bash scripts/install/install_deps.bash

#compile libwave
    mkdir -p build
    cd build
    cmake ..
    make


## Documentation

The documentation consists of an API reference as well as pages on Fundamentals,
describing concepts and theory behind the library.

It is generated using Doxygen, using the build target `make doc`.


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
