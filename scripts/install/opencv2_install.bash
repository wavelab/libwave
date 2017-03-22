#!/bin/bash
set -e  # exit on first error

echo "Installing OpenCV 2.4 ..."
sudo apt-get install -qq -y libopencv-dev python-opencv
