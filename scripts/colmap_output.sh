#!/bin/bash

# Use 'ros2 pkg prefix --share' instead of 'rospack find'
TARGET_DIRS=(
    "$(ros2 pkg prefix --share fast_livo)/Log/Colmap/images"
    "$(ros2 pkg prefix --share fast_livo)/Log/Colmap/sparse/0"
)

for dir in "${TARGET_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        rm -rf "$dir"
        echo "Removed: $dir"
    else
        echo "Not found: $dir"
    fi
done

for dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo "Created: $dir"
    else
        echo "Exists: $dir"
    fi
done
