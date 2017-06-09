#!/bin/bash
set -e  # exit on first error

# Prompt for users but not on CI
y=
if [[ "$CI" == "true" ]]; then
  y=-y
fi

echo
echo "Some of libwave's dependencies are available from a Personal Package" \
"Archive (PPA). You will be prompted to add the PPA to your sources list." \
"Note these packages are not supported by Ubuntu or the authors of libwave."
echo

# Echo the commands before running them
set -x
sudo apt-add-repository $y ppa:lkoppel/robotics
sudo apt-get update

sudo apt-get install $y cmake \
  libboost-dev \
  libyaml-cpp-dev \
  libeigen3-dev \
  libpcl-dev \
  libopencv-dev \
  libceres-dev \
  libkindr-dev

set +x
echo
echo "Done installing libwave dependencies."
