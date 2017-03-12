# libwave [![Build Status](https://api.travis-ci.org/wavelab/libwave.svg?branch=master)](https://travis-ci.org/wavelab/libwave)

This library contains reusable code for:

- Estimation
- Kinematics
- Optimization
- Vision
- and more ..


## Install

    # clone repo
    git clone https://github.com/wavelab/libwave.git
    cd libwave

    # initialize git submodules
    git submodule init
    git submodule update

    # install dependencies
    bash scripts/install_deps.bash

    # compile libwave
    mkdir -p build
    cd build
    cmake ..
    make

**Or** if you're feeling lazy you can copy this into your terminal

    curl -L https://git.io/vyKXR > install.bash && bash install.bash

Which does exactly the same thing.


## How to view docs?

We are currently using `ditto` to document the project, to see the docs perform
the following:

    sudo pip install livereload
    cd <path of libwave>/docs
    livereload
    # the docs are now served on 127.0.0.1:35729
    # open up a web-browser and type in the above IP address to view docs

LiveReload is a python app that serves the project documentation locally, it
updates itself when you update the docs. For more information on how to edit
docs see [this][edit_docs].


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
