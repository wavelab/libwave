#!/bin/bash
set -e  # exit on first error

echo "Installing Ceres ..."
sudo apt-get install -qq cmake \
                    libgoogle-glog-dev \
                    libatlas-base-dev \
                    libeigen3-dev \
                    libsuitesparse-dev \
                    libceres \
                    libceres-dev

sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687 -y
sudo apt-get update
sudo apt-get install -qq libsuitesparse-dev
