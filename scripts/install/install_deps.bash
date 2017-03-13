#!/bin/bash
set -e  # exit on first error
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo sh $SCRIPT_PATH/boost_install.bash
# sudo sh $SCRIPT_PATH/ceres_install.bash
sudo sh $SCRIPT_PATH/eigen_install.bash
sudo sh $SCRIPT_PATH/yaml_cpp_install.bash
sudo sh $SCRIPT_PATH/opencv3_install.bash
