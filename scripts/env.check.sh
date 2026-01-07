#!/usr/bin/env bash
# =============================================================================
# ESP-IDF sensor_control component structure checker
# =============================================================================

set -e

BASE_DIR="components/sensor_control"

FILES=(
    "CMakeLists.txt"
    "sensor_control.cpp"
    "tof_sensor_driver.cpp"
    "tof_sensor_driver.h"
    "include/sensor_control.h"
)

echo "Checking sensor_control component structure..."

# Create base directory
if [ ! -d "$BASE_DIR" ]; then
    echo "Creating $BASE_DIR"
    mkdir -p "$BASE_DIR"
fi

# Create include directory
if [ ! -d "$BASE_DIR/include" ]; then
    echo "Creating $BASE_DIR/include"
    mkdir -p "$BASE_DIR/include"
fi

# Create files if missing
for file in "${FILES[@]}"; do
    path="$BASE_DIR/$file"
    if [ ! -f "$path" ]; then
        echo "Creating missing file: $path"
        touch "$path"
    else
        echo "OK: $path"
    fi
done

echo
echo "sensor_control component structure is valid."
