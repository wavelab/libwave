#!/bin/sh
set -e  # exit on first error

sudo apt-get update

sudo apt-get install cmake \
                    libgoogle-glog-dev \
                    libatlas-base-dev \
                    libeigen3-dev \
                    libsuitesparse-dev \
                    libceres \
                    libceres-dev

sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev
