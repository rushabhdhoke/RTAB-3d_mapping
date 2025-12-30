#!/bin/bash

# CORRECTED RTAB-Map launch command for your D435i setup
# The issue was: your topics have /camera/camera/ prefix, not /camera/

echo "=========================================="
echo "Launching RTAB-Map with CORRECTED topics"
echo "=========================================="
echo ""
echo "Your actual camera topics:"
echo "  RGB: /camera/camera/color/image_raw"
echo "  Depth: /camera/camera/aligned_depth_to_color/image_raw"
echo "  Info: /camera/camera/color/camera_info"
echo ""

# First check if topics exist
if ! ros2 topic list | grep -q "/camera/camera/color/image_raw"; then
    echo "ERROR: Camera topics not found!"
    echo "Please start your camera driver first:"
    echo "  ros2 launch realsense2_camera rs_launch.py"
    exit 1
fi

echo "Topics found! Launching RTAB-Map..."
echo ""

ros2 launch rtabmap_launch rtabmap.launch.py \
    namespace:=rtabmap \
    frame_id:=camera_link \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    approx_sync:=true \
    qos:=1 \
    topic_queue_size:=50 \
    sync_queue_size:=50

