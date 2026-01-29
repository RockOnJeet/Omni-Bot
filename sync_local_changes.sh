#!/bin/bash
set -e

BASE=~/Omni-Bot/src

sync_patch () {
    local name=$1
    local dir=$2

    cd "$BASE/$dir"

    if git diff --quiet; then
        echo "$name: no local changes"
        return
    fi

    git diff > "$BASE/$name.local.patch"
    echo "$name: patch updated"
}

sync_patch ydlidar_ros2_driver ydlidar_ros2_driver
sync_patch YDLidar-SDK YDLidar-SDK
