# autoware_pcl_extensions

## Overview

The `autoware_pcl_extensions` is a pcl extension library. The voxel grid filter in this package works with a different algorithm than the original one.

## Design

Inner-workings / Algorithms

### Original Algorithm [1]

1. create a 3D voxel grid over the input pointcloud data
2. calculate centroid in each voxel
3. all the points are approximated with their centroid

### Extended Algorithm

1. create a 3D voxel grid over the input pointcloud data
2. calculate centroid in each voxel
3. **all the points are approximated with the closest point to their centroid**

## Inputs / Outputs

input and output are pointcloud with the datatype `pcl::PointCloud<pcl::PointXYZ>`

## Parameters

`leaf_size`: it represents the scale of each cell in 3 dimension.

It can be set by calling this function, where the scale of the cell in the axis of x, y and z are set respectively

```cpp
filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
```

## Usage

include the header file

```cpp
#include <autoware/pcl_extensions/voxel_grid_nearest_centroid.hpp>
```

init the filter with input pointcloud, set leaf size and get output result

```cpp
pcl::VoxelGridNearestCentroid<pcl::PointXYZ> filter;
filter.setInputCloud(pcl_input);
filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
filter.filter(*pcl_output);
```

## (Optional) References/External links

[1] <https://pointclouds.org/documentation/tutorials/voxel_grid.html>
