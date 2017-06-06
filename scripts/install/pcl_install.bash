#!/bin/bash
set -e  # exit on first error

echo "Installing PCL 1.8..."

UBUNTU_VERSION=`lsb_release --release | cut -f2`

if [[ "$UBUNTU_VERSION" == "14.04" || "$UBUNTU_VERSION" == "16.04" ]]; then
    # PCL 1.8 is not available from official repo in these versions
    sudo add-apt-repository -y ppa:lkoppel/robotics
    sudo apt update
fi

sudo apt-get install -qq libpcl-dev
