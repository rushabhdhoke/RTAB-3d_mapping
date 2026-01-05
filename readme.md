# RTAB-Map 3D Scene Mapping for Isaac Sim

**A ROS2-based solution for creating 3D maps using RTAB-Map and Intel RealSense D435i, designed for import into NVIDIA Isaac Sim for robot simulation environments.**

![Mapping](RTAB-3d_mapping/Image.png)

## 🎯 Project Overview

This project enables real-time 3D mapping of physical environments using RTAB-Map with an Intel RealSense D435i depth camera. The generated 3D point clouds can be exported and imported into NVIDIA Isaac Sim to create realistic simulation environments for robot navigation and testing.

### Core Objectives

- ✅ **3D Scene Mapping**: Build high-quality 3D maps using RTAB-Map
- ✅ **Real-Time Processing**: Live mapping with synchronized RGB-D camera data
- ✅ **Export-Ready**: Generate point clouds suitable for simulation environments
- 🔄 **Isaac Sim Integration**: Import 3D maps into Isaac Sim (work in progress)

## 📋 Features

- **Optimized Configuration**: Pre-configured RTAB-Map parameters for D435i camera
- **Automatic Topic Detection**: Handles RealSense camera topic namespacing
- **Noise Reduction**: Tuned parameters for cleaner 3D point clouds
- **Diagnostic Tools**: Built-in scripts for troubleshooting camera connectivity
- **ROS2 Native**: Fully integrated with ROS2 Jazzy/Humble

## 🚀 Quick Start

### Prerequisites

- **ROS2**: Jazzy or Humble distribution
- **Hardware**: Intel RealSense D435i camera
- **Software**:
  ```bash
  sudo apt install ros-$ROS_DISTRO-rtabmap-ros \
                   ros-$ROS_DISTRO-realsense2-camera
  ```

### Installation

1. **Clone this repository:**
   ```bash
   git clone <your-repo-url>
   cd rtabmap_3d_mapping
   ```

2. **Make scripts executable:**
   ```bash
   chmod +x scripts/*.sh
   ```

3. **Source ROS2:**
   ```bash
   source /opt/ros/jazzy/setup.bash  # or humble
   ```

### Usage

**Terminal 1 - Start Camera Driver:**
```bash
cd scripts
./start_camera.sh
```

**Terminal 2 - Launch RTAB-Map:**
```bash
cd scripts
./launch_rtabmap.sh
```

The RTAB-Map GUI will open automatically, and you'll see:
- Real-time camera feed
- Live 3D point cloud building
- Odometry visualization
- Loop closure detection

Move the camera slowly around your environment to build the 3D map.

## 📁 Project Structure

```
rtabmap_3d_mapping/
├── scripts/
│   ├── start_camera.sh          # RealSense D435i camera driver launcher
│   ├── launch_rtabmap.sh        # RTAB-Map launch script (main)
│   └── check_camera_topics.sh   # Camera connectivity diagnostics
├── config/
│   └── rtabmap_optimized.ini    # RTAB-Map parameter configuration
├── docs/
│   └── ISAAC_SIM_INTEGRATION.md # Isaac Sim import guide (in progress)
└── README.md                     # This file
```

## 🔧 Configuration

### Key Settings

- **Camera Topics**: `/camera/camera/color/image_raw` (RealSense default)
- **QoS**: Best Effort (qos=1) for real-time performance
- **Frame Rate**: 30 Hz RGB-D synchronized
- **Depth Range**: 0.2m - 4.0m (optimized for indoor mapping)

### Customization

Edit `config/rtabmap_optimized.ini` to adjust:
- Feature detection parameters
- Loop closure thresholds
- Point cloud filtering
- Memory management settings

## 📊 Exporting 3D Maps for Isaac Sim

### Current Workflow

1. **Build the Map**: Use RTAB-Map to map your environment
2. **Export Point Cloud**: 
   - In RTAB-Map GUI: `File > Export 3D Map`
   - Formats: `.ply`, `.pcd`, `.obj`
   - Recommended: `.ply` (Point Cloud Library format)

3. **Post-Processing** (recommended):
   - Use CloudCompare or Meshlab to:
     - Remove noise and outliers
     - Downsample if needed
     - Convert to mesh if required

### Isaac Sim Import (General Procedure)

**Note**: This is a work-in-progress. General approach:

1. **Point Cloud to Mesh Conversion**:
   - Use CloudCompare or RTAB-Map post-processing tools
   - Convert point cloud to triangulated mesh
   - Export as `.obj` or `.usd` format

2. **Import into Isaac Sim**:
   - Use Isaac Sim's USD import functionality
   - Place mesh as static geometry
   - Configure collision and physics properties
   - Add lighting and materials

3. **Robot Integration**:
   - Spawn robot model in the imported environment
   - Configure sensors and navigation
   - Run simulation

**Detailed guide**: See `docs/ISAAC_SIM_INTEGRATION.md` (coming soon)

## 🛠️ Troubleshooting

### Camera Not Detected

```bash
cd scripts
./check_camera_topics.sh
```

**Common fixes**:
- Check USB connection (use USB 3.0 port)
- Verify camera permissions: `sudo chmod 666 /dev/video*`
- Restart camera driver

### Topics Not Publishing

- Wait 5-10 seconds after starting camera driver
- Check camera is recognized: `rs-enumerate-devices`
- Verify RealSense driver is installed correctly

### Poor Map Quality

- Move camera slowly (< 1 m/s)
- Ensure good lighting
- Check depth sensor isn't blocked
- Adjust parameters in `config/rtabmap_optimized.ini`

### RTAB-Map Not Launching

- Verify ROS2 is sourced: `echo $ROS_DISTRO`
- Check RTAB-Map is installed: `ros2 pkg list | grep rtabmap`
- Review terminal output for error messages

## 📝 Example Workflow

```bash
# 1. Start camera
cd ~/ros2_ws/rtabmap_3d_mapping/scripts
source /opt/ros/jazzy/setup.bash
./start_camera.sh

# 2. In new terminal, start RTAB-Map
cd ~/ros2_ws/rtabmap_3d_mapping/scripts
source /opt/ros/jazzy/setup.bash
./launch_rtabmap.sh

# 3. Map your environment
# Move camera slowly around the space

# 4. Export map from RTAB-Map GUI
# File > Export 3D Map > Save as .ply

# 5. Post-process and import to Isaac Sim
# (See docs/ISAAC_SIM_INTEGRATION.md)
```

## 🎓 References

- [RTAB-Map Documentation](http://wiki.ros.org/rtabmap_ros)
- [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
- [ROS2 Documentation](https://docs.ros.org/)

## 🤝 Contributing

Contributions are welcome! Areas for improvement:
- Isaac Sim import automation
- Post-processing scripts
- Additional camera support
- Performance optimizations


## 👤 Author

[Rushabh Dhoke](https://github.com/rushabhdhoke)

---

**Status**: ✅ Working - RTAB-Map 3D mapping functional | 🔄 Isaac Sim integration in progress
