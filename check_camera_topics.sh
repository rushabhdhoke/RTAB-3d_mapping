#!/bin/bash

echo "=========================================="
echo "Camera Topic Diagnostics for RTAB-Map"
echo "=========================================="
echo ""

echo "1. Checking if camera topics are available..."
echo "--------------------------------------------"
ros2 topic list | grep -E "camera|image|depth" || echo "No camera topics found!"

echo ""
echo "2. Checking topic types..."
echo "--------------------------------------------"
ros2 topic list -t | grep -E "camera|image|depth" || echo "No camera topics found!"

echo ""
echo "3. Checking if required topics exist..."
echo "--------------------------------------------"
REQUIRED_TOPICS=(
  "/camera/color/image_raw"
  "/camera/aligned_depth_to_color/image_raw"
  "/camera/color/camera_info"
)

for topic in "${REQUIRED_TOPICS[@]}"; do
  if ros2 topic list | grep -q "^${topic}$"; then
    echo "✓ $topic - EXISTS"
    echo "  Publishing rate:"
    timeout 2 ros2 topic hz "$topic" 2>/dev/null || echo "    Could not determine rate"
    echo "  Message type:"
    ros2 topic type "$topic"
  else
    echo "✗ $topic - MISSING"
  fi
  echo ""
done

echo ""
echo "4. Checking frame_id in camera_info..."
echo "--------------------------------------------"
ros2 topic echo /camera/color/camera_info --once 2>/dev/null | grep -A 5 "header:" || echo "Could not read camera_info"

echo ""
echo "5. Checking TF tree (camera_link frame)..."
echo "--------------------------------------------"
ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null | head -5 || echo "TF not available or frame not found"

echo ""
echo "=========================================="
echo "Diagnostics complete!"
echo "=========================================="

