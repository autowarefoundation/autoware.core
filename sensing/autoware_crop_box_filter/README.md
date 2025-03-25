# autoware_crop_box_filter

## Overview

The `autoware_crop_box_filter` is a package that crops the input pointcloud to a specified bounding box. This is useful for reducing the computational load and improving the performance of the system.

## Design

The `autoware_crop_box_filter` is implemented as a autoware core node that subscribes to the input pointcloud, and publishes the filtered pointcloud. The bounding box is specified using the `min_point` and `max_point` parameters.

## Inputs / Outputs

### Input

| Name             | Type                            | Description      |
| ---------------- | ------------------------------- | ---------------- |
| `~/input/points` | `sensor_msgs::msg::PointCloud2` | reference points |

### Output

| Name                 | Type                                 | Description          |
| -------------------- | ------------------------------------ | -------------------- |
| `~/output/points`    | `sensor_msgs::msg::PointCloud2`      | filtered points      |
| `~/crop_box_polygon` | `geometry_msgs::msg::PolygonStamped` | bounding box polygon |

## Parameters

### Launch file Parameters

| Name                         | Type   | Default Value | Description                                  |
| ---------------------------- | ------ | ------------- | -------------------------------------------- |
| `input_frame`                | string | " "           | the frame id in which filtering is performed |
| `output_frame`               | string | " "           | output frame id of the filtered points       |
| `input_pointcloud_frame`     | string | " "           | frame id of input pointcloud                 |
| `max_queue_size`             | int    | 5             | max buffer size of input/output topics       |
| `crop_box_filter_param_file` | string | " "           | path to the parameter file for the node      |

### Node Parameters

| Name       | Type   | Default Value | Description                                                                              |
| ---------- | ------ | ------------- | ---------------------------------------------------------------------------------------- |
| `min_x`    | double | -5.0          | minimum x value of the crop box                                                          |
| `min_y`    | double | -5.0          | minimum y value of the crop box                                                          |
| `min_z`    | double | -5.0          | minimum z value of the crop box                                                          |
| `max_x`    | double | 5.0           | maximum x value of the crop box                                                          |
| `max_y`    | double | 5.0           | maximum y value of the crop box                                                          |
| `max_z`    | double | 5.0           | maximum z value of the crop box                                                          |
| `negative` | bool   | true          | if true, points inside the box are removed, otherwise points outside the box are removed |

## Usage

### 1.publish static tf from input pointcloud to target frame that is used for filtering

```cpp
ros2 run tf2_ros static_transform_publisher 2.0 3.2 1.3 0 0 0 1  velodyne_top_base_link  base_link
```

### 2.launch node

```cpp
ros2 launch autoware_crop_box_filter crop_box_filter_node.launch.xml
```

### 3. launch rviz2 and AWSIM
