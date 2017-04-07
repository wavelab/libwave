#!/bin/bash
set -e  # exit on first error

install_dependencies_14() {
    sudo apt-get -y install -qq \
    libboost-all-dev \
    libflann-dev \
    libeigen3-dev \
    libusb-1.0-0-dev \
	libvtk6 \
	libvtk6-java \
	python-vtk6 \
	tcl-vtk6 \
	libqt5opengl5
}

install_dependencies_16() {
    sudo apt-get -y install -qq \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev \
    libopenni-dev \
    libpcl1.7 \
    libqhull-dev \
    libvtk6-dev \
    libvtk6-qt-dev
}

echo "Installing PCL ..."

UBUNTU_VERSION=`lsb_release --release | cut -f2`

if [ $UBUNTU_VERSION == "16.04" ]; then
    install_dependencies_16
    sudo apt-get -y install -qq libpcl-dev

elif [ $UBUNTU_VERSION == "14.04" ]; then
    install_dependencies_14
    sudo add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl
    sudo apt-get -y update
    sudo apt-get -y install -qq libpcl-all
fi
