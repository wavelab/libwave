#!/bin/bash
set -e  # exit on first error
UBUNTU_VERSION=`lsb_release --release | cut -f2`
SRC_PREFIX_PATH="/usr/local/src/"
CERES_VERSION=1.12.0


install_dependencies() {
    apt-get update -qq
    apt-get install -qq -y cmake \
                           libgoogle-glog-dev \
                           libatlas-base-dev \
                           libeigen3-dev \
                           libsuitesparse-dev
}

install_suitesparse_fix() {
    add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687 -y
    apt-get update -qq
    apt-get install -qq libsuitesparse-dev
}

install_ceres_solver() {
    mkdir -p $SRC_PREFIX_PATH
    cd $SRC_PREFIX_PATH

    if [ ! -d ceres-solver ]; then
        # clone ceres-solver if directory does not already exist
       git clone https://github.com/ceres-solver/ceres-solver.git
    else
        # otherwise, fetch to ensure we have the newest tags
       git -C ceres-solver fetch
    fi

    # go into ceres-solver repo and prepare for build
    cd ceres-solver
    git checkout tags/${CERES_VERSION}
    mkdir -p build
    cd build
    cmake ..

    # compile and install
    make -j$(nproc)
    # Test, but skip the slowest tests
    make test ARGS="-E 'bundle_adjustment|covariance|rotation'"
    make install
}

# MAIN
if [ $UBUNTU_VERSION == "16.04" ]; then
    install_dependencies
    install_ceres_solver

elif [ $UBUNTU_VERSION == "14.04" ]; then
    install_dependencies
    install_suitesparse_fix
    install_ceres_solver
fi
