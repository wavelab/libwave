#!/bin/bash
set -e  # exit on first error

DEPS_DIR="/tmp/wave_dependencies"

main()
{
    install_gtsam
}

make_with_progress()
{
    if [ -z "$CONTINUOUS_INTEGRATION" ]; then
        local awk_arg="-W interactive"
    fi
    # Run make, printing a character for every 10 lines
    make "$@" | awk ${awk_arg} 'NR%5==1 { printf ".", $0}'
    echo "done"
}

install_gtsam()
{
    GTSAM_DIR="gtborg-gtsam-250a03943591"
    FILE="250a03943591.zip"
    URL="https://bitbucket.org/gtborg/gtsam/get/250a03943591.zip"

    if (ldconfig -p | grep -q libgtsam.so.4 ); then
        echo "gtsam is already installed."
    else
        echo "Installing gtsam ..."
        mkdir -p "$DEPS_DIR"
        cd "$DEPS_DIR"
        wget "$URL"
        unzip "$FILE"
        rm -rf "$FILE"

        cd "$GTSAM_DIR"
        mkdir -p BUILD
        cd BUILD
        cmake ..

        make_with_progress -j$(nproc)
        sudo make install > /dev/null
    fi
}

main
