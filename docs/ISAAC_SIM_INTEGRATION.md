# Isaac Sim Integration Guide

**Status**: 🔄 Work in Progress

This document outlines the procedure for importing RTAB-Map 3D scenes into NVIDIA Isaac Sim for robot simulation.

## Overview

The goal is to convert RTAB-Map's 3D point clouds into simulation environments where robots can navigate and interact. This enables:
- Testing navigation algorithms in realistic environments
- Training AI models with accurate 3D representations
- Simulating robot behavior before real-world deployment

## General Workflow

```
RTAB-Map → Export Point Cloud → Post-Processing → Mesh Conversion → Isaac Sim Import → Robot Simulation
```

## Step 1: Export from RTAB-Map

### Export Point Cloud

1. **Build your map** in RTAB-Map (see main README)
2. **Export the 3D map**:
   - RTAB-Map GUI: `File > Export 3D Map`
   - Choose format:
     - **`.ply`** (Recommended) - Point Cloud Library format
     - **`.pcd`** - PCL format
     - **`.obj`** - Wavefront OBJ format

3. **Save location**: Note where you saved the exported file

### Export Settings

- **Coordinate System**: Ensure consistent frame reference
- **Resolution**: Balance between detail and file size
- **Include Colors**: Enable if you want colored meshes in Isaac Sim

## Step 2: Post-Processing

### Using CloudCompare (Recommended)

1. **Install CloudCompare**:
   ```bash
   sudo apt install cloudcompare
   ```

2. **Open exported point cloud**:
   - File > Open > Select your `.ply` file

3. **Clean the point cloud**:
   - Remove outliers: `Edit > Outlier removal`
   - Remove noise: `Edit > Noise filter`
   - Remove ground if needed: `Edit > Segment`

4. **Downsample** (if too dense):
   - Edit > Subsample
   - Choose voxel grid or random sampling

5. **Convert to Mesh** (Required for Isaac Sim):
   - Edit > Mesh > Delaunay 2.5D (for surfaces)
   - Or Edit > Mesh > Poisson surface reconstruction
   - Export mesh: File > Save > Choose `.obj` or `.stl`

### Using Meshlab

Alternative post-processing tool:
```bash
sudo apt install meshlab
```

- Import point cloud
- Filters > Remeshing > Surface Reconstruction: Poisson
- Export as `.obj` or `.stl`

## Step 3: Format Conversion for Isaac Sim

Isaac Sim primarily uses **USD** (Universal Scene Description) format.

### Option A: Direct USD Export

If using CloudCompare with USD support:
- File > Save > Choose `.usd` format
- This is the native Isaac Sim format

### Option B: Convert OBJ to USD

Using Isaac Sim tools or USD Python API:

```python
# Example: Convert OBJ to USD using USD Python API
from pxr import Usd, UsdGeom

# Load OBJ and convert to USD
# (Full implementation depends on your setup)
```

### Option C: Use Isaac Sim Importer

1. Open Isaac Sim
2. File > Import > Mesh
3. Select your `.obj` or `.stl` file
4. Isaac Sim will convert it to USD internally

## Step 4: Import into Isaac Sim

### Method 1: GUI Import

1. **Open Isaac Sim**
2. **Create new stage** or open existing
3. **Import mesh**:
   - File > Import > Mesh
   - Select your processed mesh file
   - Adjust import settings

4. **Position and scale**:
   - Use transform tools to position correctly
   - Ensure scale matches your robot's dimensions

### Method 2: Programmatic Import

Using Isaac Sim Python API:

```python
import omni.usd
from pxr import Usd, UsdGeom

# Create stage
stage = omni.usd.get_context().get_stage()

# Import mesh as reference
mesh_prim = stage.DefinePrim("/World/EnvironmentMesh")
mesh_prim.GetReferences().AddReference("path/to/your/mesh.usd")

# Or load directly
omni.usd.get_context().open_stage("path/to/your/mesh.usd")
```

## Step 5: Configure Physics and Collision

### Set Up Collision

1. **Select imported mesh** in Isaac Sim
2. **Physics properties**:
   - Enable "Collider" checkbox
   - Choose collision shape (mesh, convex hull, etc.)
   - Set collision layers

3. **Material properties**:
   - Set friction and restitution
   - Add visual materials if needed

### Make Static

- Ensure mesh is marked as **Static** in physics properties
- This prevents unnecessary physics calculations

## Step 6: Robot Integration

### Spawn Robot

1. **Import robot model** (USD format)
2. **Position robot** in the imported environment
3. **Configure sensors**:
   - Add cameras, LiDAR, etc.
   - Match sensor parameters to real robot

### Navigation Setup

1. **Configure Nav2** or navigation stack
2. **Set up path planning** in the environment
3. **Test robot movement**

## Current Limitations & TODO

### Known Issues

- [ ] Automatic USD conversion pipeline
- [ ] Direct RTAB-Map database to USD export
- [ ] Mesh quality optimization for simulation
- [ ] Automatic collision mesh generation
- [ ] Scale and coordinate frame alignment automation

### Future Enhancements

- [ ] Automated export pipeline script
- [ ] Real-time streaming from RTAB-Map to Isaac Sim
- [ ] Integration with RTAB-Map's post-processing tools
- [ ] Automatic material assignment
- [ ] Lighting optimization for imported scenes

## Tools & Resources

### Required Software

- **CloudCompare**: Point cloud processing
- **Meshlab**: Alternative mesh processing
- **NVIDIA Isaac Sim**: Robot simulation environment
- **USD Tools**: For format conversion

### Helpful Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
- [USD Documentation](https://openusd.org/release/index.html)
- [CloudCompare Tutorials](https://www.cloudcompare.org/doc/wiki/index.php?title=Tutorials)

## Example Pipeline Script (Concept)

```bash
#!/bin/bash
# Future: Automated export pipeline

# 1. Export from RTAB-Map
# (User exports manually or via RTAB-Map CLI)

# 2. Post-process
cloudcompare -O map.ply -REMOVE_OUTLIER -SAVE_MESH map_clean.obj

# 3. Convert to USD
isaac_sim_convert_obj_to_usd map_clean.obj map_clean.usd

# 4. Import to Isaac Sim
isaac_sim --stage map_clean.usd
```

## Troubleshooting

### Mesh Too Complex

- Reduce point cloud density before meshing
- Use convex hull for collision instead of mesh
- Simplify mesh using Meshlab decimation

### Scale Issues

- Verify unit system (meters vs. centimeters)
- Use Isaac Sim's scale tool to adjust
- Check RTAB-Map export units

### Import Errors

- Verify file format compatibility
- Check USD version compatibility
- Ensure mesh is manifold (no holes)

### Performance Issues

- Reduce mesh complexity
- Use level-of-detail (LOD) meshes
- Optimize collision meshes separately

---

**Note**: This integration is actively being developed. Contributions and feedback are welcome!
