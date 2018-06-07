#!/bin/bash
set -e  # exit on first error

echo
echo "Some of libwave's dependencies are available from a Personal Package" \
"Archive (PPA). You will be prompted to add the PPA to your sources list." \
"Note these packages are not supported by Ubuntu or the authors of libwave."
echo

# Echo the commands before running them
set -x
sudo add-apt-repository ppa:lkoppel/robotics-wavelab
sudo apt-get update

sudo apt-get install cmake \
  libboost-dev \
  libyaml-cpp-dev \
  libeigen3-dev \
  libpcl1.8-dev \
  libopencv-dev \
  libopencv-core-dev \
  libceres-wavelab-dev

set +x
echo
echo "Done installing libwave dependencies."
