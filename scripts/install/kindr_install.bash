#!/bin/bash
set -e  # exit on first error
KINDR_VER="1.0.4"
KINDR_URL="https://github.com/ethz-asl/kindr/archive/$KINDR_VER.zip"

download_kindr() {
    mkdir -p /usr/local/src/kindr
    cd /usr/local/src/kindr
    wget $KINDR_URL -O kindr-$KINDR_VER.zip
    unzip kindr-$KINDR_VER.zip
    cd -
}

install_kindr() {
    # compile and install kindr
    cd /usr/local/src/kindr
    cd kindr-$KINDR_VER
    mkdir -p build
    cd build
    cmake ..
    sudo make install
    cd -
}

# MAIN
echo "Installing Kindr ..."
download_kindr
install_kindr
