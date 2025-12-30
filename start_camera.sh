#!/bin/bash

# Script to start RealSense D435i camera driver
echo "=========================================="
echo "Starting RealSense D435i Camera Driver"
echo "=========================================="
echo ""

# Check if camera is connected
if ! rs-enumerate-devices &>/dev/null; then
    echo "ERROR: Camera not detected!"
    echo "Please check USB connection."
    exit 1
fi

echo "Camera detected. Starting driver..."
echo ""
echo "Starting with aligned_depth enabled for RTAB-Map..."
echo "Press Ctrl+C to stop"
echo ""

# Start RealSense driver with proper settings for RTAB-Map
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    enable_depth:=true \
    enable_color:=true \
    enable_infra1:=false \
    enable_infra2:=false \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30

