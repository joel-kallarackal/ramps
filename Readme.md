# Incline Detection

Raw point clouds are clustered via their normals and then planes with various orientations are found.

## Setting up  
The package uses Open3D library for processing point clouds  
```
pip install open3d
```

## Running the code  
```
ros2 run ramps ramp_detection.py
```