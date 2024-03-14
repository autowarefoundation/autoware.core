# Autoware Node

## Overview
<!-- cspell:ignore registrate -->
Autoware Node is an Autoware.Core package designed to provide a base class for all future nodes in the system. It provides ability to registrate node to _Autoware_control_center_ (ACC), report node state, publish heartbeat and subscribe to monitored topics. It also inheritates all lifecycle control capabilities of the base class [LifecycleNode](https://docs.ros2.org/latest/api/rclcpp_lifecycle/classrclcpp__lifecycle_1_1LifecycleNode.html)

## Usage

You can use _autoware_node_ as a base class for any node in Autoware.Core system. There is an example package _test_node_ which shows how _autoware_node_ communicate with ACC and other _autoware_nodes_. You can check it for more information.

## Design
<!-- cspell:ignore  ROS2 -->
_Autoware_node_ inherits from  ROS2 [_lifecycle_node_](https://design.ros2.org/articles/node_lifecycle.html) and has all basic functions of it.

Below are the main add-ons and how they work.

### Registration

After startup each _autoware_node_ tries to register itself to ACC via a service call of _AutowareNodeRegister_. It happens in the dedicated timer. The timer will stop after the successful registration.

### De-registration

If _autoware_node_ receives a request to it's _AutowareControlCenterDeregister_ service. It will disable the flag _registered_ and it will startup timer which control registration client.

### Error state

_Autoware_node_ has `send_state` method to send it's state to ACC via _AutowareNodeError_ service. You need to provide `AutowareNodeState` and log message as parameters to the method.

### Heartbeat

The heartbeat publisher is configured to publish the heartbeat message each 200 ms. You can change it by the `period` parameter during the launch of _autoware_node_. Be aware that you will also need to configure ACC accordingly.

Heartbeat functionality is based on ros2 [software_watchdogs](https://github.com/ros-safety/software_watchdogs) package.

### Monitored subscription

_Autoware_node_ provides the `create_monitored_subscription` method. It wraps around a standard `create_subscription` method and adds a function to monitor a frequency of messages received in the topic. If it violates the condition provided in the `hz` parameter of the method _autoware_node_ will send an error state to the ACC.
