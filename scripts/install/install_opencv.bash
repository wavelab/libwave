#!/bin/bash
set -e  # exit on first error

DEPS_DIR="/tmp/wave_dependencies"

main()
{
    install_opencv
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

install_opencv()
{
    VERSION="3.2.0"
    URL="https://github.com/opencv/opencv/archive/$VERSION.zip"
    OPENCV_DIR="opencv-$VERSION"
    FILE="$VERSION.zip"

    pkg-config --atleast-version=$VERSION opencv
    if [ "$?" == "0" ]; then
        echo "opencv version $(pkg-config --modversion opencv) is already installed."
    else
        echo "Installing opencv version $VERSION ..."
        mkdir -p "$DEPS_DIR"
        cd "$DEPS_DIR"
        wget "$URL"
        unzip "$FILE"
        rm -rf "$FILE"

        cd "$OPENCV_DIR"
        mkdir -p BUILD
        cd BUILD
        cmake ..

        make_with_progress -j$(nproc)
        sudo make install > /dev/null
    fi
}

main
