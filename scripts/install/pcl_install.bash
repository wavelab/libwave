#!/bin/bash
set -e  # exit on first error

install_dependencies() {
    sudo apt-get -y install -qq \
        libboost-all-dev \
        libflann-dev \
        libvtk6-dev \
        libeigen3-dev \
        libusb-1.0-0-dev
}

echo "Installing PCL ..."

install_dependencies

sudo add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get -y update
sudo apt-get -y install -qq libpcl-all
