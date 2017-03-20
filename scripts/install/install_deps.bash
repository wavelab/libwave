#!/bin/bash
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo apt-get clean > /dev/null
sudo apt-get update > /dev/null

install_dependency() {
    echo "Installing $1 ..."
    sudo bash $2 &>install.log

    if [[ $? -ne 0 ]]; then
        echo "Failed to install [$1]"
        echo ""
        echo "Error Log:"
        echo "--------------------------------------------------"
        cat install.log
        echo ""
        exit -1
    fi
}

install_dependency "Boost" $SCRIPT_PATH/boost_install.bash
install_dependency "Ceres" $SCRIPT_PATH/ceres_install.bash
install_dependency "Eigen" $SCRIPT_PATH/eigen_install.bash
install_dependency "Yaml-CPP" $SCRIPT_PATH/yaml_cpp_install.bash
install_dependency "OpenCV 3.0" $SCRIPT_PATH/opencv3_install.bash
