#!/bin/bash
set -e

BASE=~/Omni-Bot/src

git submodule update --init --recursive

apply_patch () {
    local name=$1
    local dir=$2
    local patch="$BASE/$name.local.patch"

    if [ ! -f "$patch" ]; then
        echo "$name: no patch, skipping"
        return
    fi

    cd "$BASE/$dir"
    git reset --hard
    git clean -fd
    git apply --check "$patch"
    git apply "$patch"
    echo "$name: patch applied"
}

apply_patch ydlidar_ros2_driver ydlidar_ros2_driver
apply_patch YDLidar-SDK YDLidar-SDK
