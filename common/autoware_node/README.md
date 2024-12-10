# Autoware Node

## Abbreviations

- **AN:** Autoware Node

## Overview

AN is an `autoware.core` package designed to provide a base class for all future nodes in the
system.
It also inherits all lifecycle control capabilities of the base
class [LifecycleNode](https://docs.ros2.org/latest/api/rclcpp_lifecycle/classrclcpp__lifecycle_1_1LifecycleNode.html)

## Usage

Check the [autoware_test_node](../../demos/autoware_test_node/README.md) package for an example of how to use `autoware::Node`.

## Design

### Lifecycle

AN inherits from ROS 2 [rclcpp_lifecycle::LifecycleNode](https://design.ros2.org/articles/node_lifecycle.html) and has
all the basic functions of it.
