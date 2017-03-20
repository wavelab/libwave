#!/bin/bash
set -e  # exit on first error
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo apt-get clean > /dev/null
sudo apt-get update > /dev/null

INSTALL_ARRAY=(
    "sudo bash $SCRIPT_PATH/boost_install.bash >install.log"
    "sudo bash $SCRIPT_PATH/ceres_install.bash >install.log"
    "sudo bash $SCRIPT_PATH/eigen_install.bash >install.log"
    "sudo bash $SCRIPT_PATH/yaml_cpp_install.bash >install.log"
    "sudo bash $SCRIPT_PATH/opencv3_install.bash >install.log"
)

for CMD in "${INSTALL_ARRAY[@]}"
do
    bash -c "$CMD"
done
