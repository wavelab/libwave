#!/bin/bash
set -e  # exit on first error

install_dependencies() {
    sudo apt-get -y install -qq \
        libflann-dev \
        libvtk6-dev
}

echo "Installing PCL ..."

SRC_DIR="/usr/local/src"
PCL_VERSION='1.8.0'
PCL_FILE="pcl-$PCL_VERSION"
PCL_DIR="pcl-$PCL_FILE"
PCL_URL="https://github.com/PointCloudLibrary/pcl/archive/pcl-$PCL_VERSION.tar.gz"

install_dependencies
# TODO: find a better way to check if already installed from source
if [ -e "/usr/local/lib/libpcl_2d.so.$PCL_VERSION" ]; then
    echo "PCL version $PCL_VERSION already installed"
else
    echo "Installing PCL version $PCL_VERSION ..."
    mkdir -p "$SRC_DIR"
    cd "$SRC_DIR"
    if [[ ! -d "$PCL_DIR" ]]; then
        wget "$PCL_URL"
        tar -xf "$PCL_FILE.tar.gz"
        rm -rf "$PCL_FILE.tar.gz"
    fi
    cd "$PCL_DIR"
    mkdir -p build
    cd build

    PCL_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11"
    if [ -n "$CONTINUOUS_INTEGRATION" ]; then
        # Disable everything unneeded for a faster build
        PCL_CMAKE_ARGS="${PCL_CMAKE_ARGS} \
        -DWITH_CUDA=OFF -DWITH_DAVIDSDK=OFF -DWITH_DOCS=OFF \
        -DWITH_DSSDK=OFF -DWITH_ENSENSO=OFF -DWITH_FZAPI=OFF \
        -DWITH_LIBUSB=OFF -DWITH_OPENGL=OFF -DWITH_OPENNI=OFF \
        -DWITH_OPENNI2=OFF -DWITH_QT=OFF -DWITH_RSSDK=OFF -DWITH_VTK=OFF \
        -DBUILD_CUDA=OFF -DBUILD_GPU=OFF -DBUILD_surface=OFF \
        -DBUILD_ml=OFF -DBUILD_io=OFF -DBUILD_geometry=OFF \
        -DBUILD_tracking=OFF"
    fi

    cmake .. ${PCL_CMAKE_ARGS} > /dev/null

    echo "Building $PCL_FILE"
    make_with_progress -j4

    sudo make install > /dev/null
    echo "PCL installed successfully"
fi
