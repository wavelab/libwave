#!/bin/sh
set -e  # exit on first error

# download gtest source
apt-get install libgtest-dev -y

# build gtest
cd /usr/src/gtest
mkdir -p build
cd build
cmake ..
make

# install gtest
cp -a libgtest_main.a libgtest.a /usr/lib/
