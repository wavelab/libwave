#!/bin/bash
set -e  # exit on first error
OPENCV_URL=https://github.com/opencv/opencv/archive/3.2.0.zip


install_dependencies() {
    apt-get -y install -qq \
        libopencv-dev \
        build-essential \
        cmake \
        git \
        libgtk2.0-dev \
        pkg-config \
        python-dev \
        python-numpy \
        libdc1394-22 \
        libdc1394-22-dev \
        libjpeg-dev \
        libpng12-dev \
        libtiff*-dev \
        libjasper-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libxine2-dev \
        libgstreamer0.10-dev \
        libgstreamer-plugins-base0.10-dev \
        libv4l-dev \
        libtbb-dev \
        libqt4-dev \
        libfaac-dev \
        libmp3lame-dev \
        libopencore-amrnb-dev \
        libopencore-amrwb-dev \
        libtheora-dev \
        libvorbis-dev \
        libxvidcore-dev \
        x264 \
        v4l-utils \
        unzip
}

download_opencv() {
    mkdir -p /usr/local/src/opencv
    cd /usr/local/src/opencv
    if [ ! -d opencv ]; then
        wget $OPENCV_URL -O opencv3.2.zip
        unzip -qq opencv3.2.zip
        rm opencv3.2.zip
    fi
    cd -
}

install_opencv() {
    # compile and install opencv
    cd /usr/local/src/opencv/opencv3.2
    mkdir -p build
    cd build
    cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D WITH_TBB=ON \
        -D WITH_V4L=ON \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON ..
    make -j$(nproc)

    # THIS WILL CAUSE PROBLEMS WITH BUILDS THAT DEPEND ON OPENCV 2
    # THE REASON IS /usr/local/lib has precedence over /usr/lib
    make install
}

# MAIN
install_dependencies
download_opencv
install_opencv
