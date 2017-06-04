#!/bin/bash
set -e  # exit on first error

echo "Installing OpenCV3 from a Personal Package Archive..."

sudo add-apt-repository -y ppa:lkoppel/opencv
sudo apt-get -q update
sudo apt-get install -y -q libopencv-dev
