# Quick Start Guide

## 🚀 Getting Started

### Step 1: Start Camera Driver

**Terminal 1:**
```bash
cd ~/ros2_ws/rtabmap_3d_mapping/scripts
source /opt/ros/jazzy/setup.bash  # or humble
./start_camera.sh
```

Wait for camera to initialize (5-10 seconds).

### Step 2: Launch RTAB-Map

**Terminal 2 (new terminal):**
```bash
cd ~/ros2_ws/rtabmap_3d_mapping/scripts
source /opt/ros/jazzy/setup.bash  # or humble
./launch_rtabmap.sh
```

## ✅ Verify It's Working

1. RTAB-Map GUI should open automatically
2. You should see camera feed in the GUI
3. "Current image id" should show numbers (not "Unknown")
4. 3D map should start building as you move the camera

## 🔍 Troubleshooting

If camera topics not found:
```bash
cd ~/ros2_ws/rtabmap_3d_mapping/scripts
./check_camera_topics.sh
```

## 📁 File Structure

```
rtabmap_3d_mapping/
├── scripts/
│   ├── start_camera.sh        ← Use this first
│   ├── launch_rtabmap.sh      ← Use this second
│   └── check_camera_topics.sh ← Diagnostic tool
├── config/
│   └── rtabmap_optimized.ini  ← Parameter reference
└── README.md
```

## 🎯 What Changed?

- ✅ All files organized under `rtabmap_3d_mapping/`
- ✅ Same working command preserved
- ✅ All path references updated correctly
- ✅ Clean workspace root
