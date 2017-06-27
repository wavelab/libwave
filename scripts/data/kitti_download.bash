#!/bin/bash
DATA_PATH=$HOME/data
KITTI_URL=http://kitti.is.tue.mpg.de/kitti

DATASETS=(
    "data_odometry_calib.zip" \
    "data_odometry_gray.zip" \
    "data_odometry_poses.zip"
)

set_download_cmd() {
    # check for wget or curl as downloaders
    if type "wget" &> /dev/null; then
        DOWNLOAD_CMD='wget';
    elif type "curl" &> /dev/null; then
        DOWNLOAD_CMD='curl -O';
    else
        echo "Error! Need either Wget or cURL to download kitti datasets!";
        exit -1;
    fi
}

setup_dirs_and_permissions() {
    # check if system has wget or curl, and set download command
    set_download_cmd

    echo "Creating dataset path at [$DATA_PATH]"
    mkdir -p $DATA_PATH

    echo "Setting $USER the owner of [$DATA_PATH]"
    chown $USER:$USER $DATA_PATH
}

download_and_unpack() {
    cd $DATA_PATH
    for DATA in ${DATASETS[@]}; do
        echo "Downloading [$KITTI_URL/$DATA]"

        # download
        if [ ! -f $DATA ]; then
            $DOWNLOAD_CMD $KITTI_URL/$DATA
        fi

        # unpack
        unzip -o $DATA
    done
}


# MAIN
echo "You are about to download large amounts of data (40+GB) into [$DATA_PATH]"
read -r -p "Continue? [y/N] " RESPONSE
case "$RESPONSE" in
    [yY][eE][sS]|[yY]) 
        setup_dirs_and_permissions
        download_and_unpack
        ;;
    *)
        echo "Terminated!";
        ;;
esac
