#!/bin/bash
set -e  # exit on first error
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo apt-get clean
sudo apt-get update

sudo bash $SCRIPT_PATH/boost_install.bash
# sudo bash $SCRIPT_PATH/ceres_install.bash
sudo bash $SCRIPT_PATH/eigen_install.bash
sudo bash $SCRIPT_PATH/yaml_cpp_install.bash
sudo bash $SCRIPT_PATH/kindr_install.bash
sudo bash $SCRIPT_PATH/opencv3_install.bash
