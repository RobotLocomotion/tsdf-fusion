# Volumetric TSDF Fusion of Multiple Depth Maps

![Teaser](teaser.jpg?raw=true)

CUDA/C++ code to fuse multiple registered depth maps into a projective truncated signed distance function (TSDF) voxel volume, which can then be used to create high quality 3D surface meshes and point clouds. Tested on Ubuntu 14.04 and 16.04.

Looking for an older version? See [here](old-version).

This repository is a part of [Andy's Code Collection](http://andyzeng.github.io/).

## Change Log
* **Nov. 1, 2017.** Bug fix: `tsdf2mesh.m` now properly generates a mesh in camera coordinates instead of voxel coordinates.
* **Oct. 30, 2017.** Notice: changed default weight threshold for `SaveVoxelGrid2SurfacePointCloud` in demo code to enable creating point cloud visualizations with only one depth frame.
* **Aug. 30, 2017.** Bug fix: remove deprecated offsets from surface distance compute during integration.

## Requirements
 * NVIDA GPU with [CUDA](https://developer.nvidia.com/cuda-downloads) support
 * [OpenCV](http://opencv.org/) (tested with OpenCV 2.4.11)

## Demo
This demo fuses 50 registered depth maps from directory `data/rgbd-frames` into a projective TSDF voxel volume, and creates a 3D surface point cloud `tsdf.ply`, which can be visualized with a 3D viewer like [Meshlab](http://www.meshlab.net/).

**Note**: Input depth maps should be saved in format: 16-bit PNG, depth in millimeters.

```shell
./compile.sh # compiles demo executable
./demo # 3D point cloud saved to tsdf.ply and voxel grid saved to tsdf.bin
```

[Optional] This demo also saves the computed voxel volume into a binary file `tsdf.bin`. Run the following script in Matlab to create a 3D surface mesh `mesh.ply`, which can be visualized with [Meshlab](http://www.meshlab.net/).

```matlab
tsdf2mesh; % 3D mesh saved to mesh.ply
```

## Usage in Spartan
 The executable `demo` accepts two command line arguments. The first is the full path to the the folder where the depth images are stored, the second is the path to the `camera-intrinsics.txt` file. The images folders structure should be
 
 ```
 images
  -- 000001_depth.png
  -- 000001_pose.txt
  .
  .
 ```
 
 The executable outputs two files, `tsdf.bin` and `tsdf_pointcloud.ply`, both in the `images` folder that was passed in. `tsdf.bin` is a binary file containing the actual TSDF, can be converted to a mesh using code in `spartan`. `tsdf_pointcloud.ply` is a pointcloud sampled from the TSDF implicit surface and encoded as a ply file.

## Seen in
 * [3DMatch: Learning Local Geometric Descriptors from RGB-D Reconstructions (CVPR 2017)](http://3dmatch.cs.princeton.edu/)
 * [Semantic Scene Completion from a Single Depth Image (CVPR 2017)](http://sscnet.cs.princeton.edu/)
 * [Deep Sliding Shapes for Amodal 3D Object Detection in RGB-D Images (CVPR 2016)](http://dss.cs.princeton.edu/)

## References
 * [A Volumetric Method for Building Complex Models from Range Images (SIGGRAPH 1996)](https://graphics.stanford.edu/papers/volrange/volrange.pdf)
 * [KinectFusion: Real-Time Dense Surface Mapping and Tracking (ISMAR 2011)](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf)
 * [Scene Coordinate Regression Forests for Camera Relocalization in RGB-D Images (CVPR 2013)](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/RelocForests.pdf)
