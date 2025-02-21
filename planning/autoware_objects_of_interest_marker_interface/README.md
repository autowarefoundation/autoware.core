# Objects Of Interest Marker Interface

## Overview

`autoware_objects_of_interest_marker_interface` is a collection of object visualization function packages.

## Design

This package implement a library to manage and visualize the object information by construct and publish it as marker array to rviz.

For a object to be visualized, it has three import characteristics.

- pose the position of the object
- shape the shape of the Bounding box of the object
- color the color of the Bounding box of the object

## Usage

### init

include the header file to use then init the library

```cpp
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>

autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface
    objects_of_interest_marker_interface_{this, "obstacle_cruise_planner"};
```

### insert

insert object information to the 'objects_of_interest_marker_interface' manager

```cpp
using autoware::objects_of_interest_marker_interface::ColorName;
objects_of_interest_marker_interface_.insertObjectData(
    stopped_obstacle.pose, stopped_obstacle.shape, ColorName::RED);
```

### publish

publish object information to the rviz to visualize

```cpp
objects_of_interest_marker_interface_.publishMarkerArray();
```
