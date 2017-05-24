#!/bin/bash
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo apt-get clean > /dev/null
sudo apt-get update > /dev/null
sudo apt-get install cmake g++ > /dev/null

install_dependency() {
    echo -n "Installing $1"

    if [[ $- == *i* ]]; then # If shell is interactive
    # Turn off buffering in awk (so dots print right away)
        local awk_arg="-W interactive"
    fi

    # Run the install script. Print a dot for every 10 lines of output and save
    # the full output to a log file
    sudo bash $2 |& tee install.log \
        |& awk ${awk_arg} 'NR%10==1 { printf "."} END{ print ""}'

    # Note: ${PIPESTATUS[0]} is $? of the first command ("sudo bash $2")
    if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
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
install_dependency "Yaml-CPP" $SCRIPT_PATH/yaml_cpp_install.bash
install_dependency "Ceres" $SCRIPT_PATH/ceres_install.bash
install_dependency "Eigen" $SCRIPT_PATH/eigen_install.bash
install_dependency "Kindr" $SCRIPT_PATH/kindr_install.bash
install_dependency "OpenCV 3.2" $SCRIPT_PATH/opencv3_install.bash
install_dependency "PCL 1.7.2" $SCRIPT_PATH/pcl_install.bash
