#!/bin/bash
set -e  # exit on first error

UBUNTU_VERSION=`lsb_release --release | cut -f2`

if [[ "$UBUNTU_VERSION" == "14.04" || "$UBUNTU_VERSION" == "16.04" ]]; then
    # Ceres 1.12 is not available in official repos until zesty
    # For these earlier versions, use a ppa
    sudo add-apt-repository -y ppa:lkoppel/ceres
fi

sudo apt-get -q update

# Install the package and all dependencies
sudo apt-get install -y -q libceres-dev
