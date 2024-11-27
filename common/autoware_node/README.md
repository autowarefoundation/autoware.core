# Autoware Node

## Abbreviations

- **ACC:** Autoware Control Center
- **AN:** Autoware Node
- **HB:** Heartbeat

## Overview

AN is an `autoware.core` package designed to provide a base class for all future nodes in the
system.
It provides ability to register node to ACC, report node state, publish heartbeats.
It also inherits all lifecycle control capabilities of the base
class [LifecycleNode](https://docs.ros2.org/latest/api/rclcpp_lifecycle/classrclcpp__lifecycle_1_1LifecycleNode.html)

## Usage

Check the [autoware_test_node](../autoware_test_node/README.md) package for an example of how to use `autoware::Node`.

## Design

### Lifecycle

AN inherits from ROS 2 [rclcpp_lifecycle::LifecycleNode](https://design.ros2.org/articles/node_lifecycle.html) and has
all the basic functions of it.

### Registration

Relevant parameters:

| Parameter                  | Function                                                               |
| -------------------------- | ---------------------------------------------------------------------- |
| `period_timer_register_ms` | The interval at which the AN periodically attempts to register itself. |

Upon startup, AN starts a registration timer and periodically attempts to register itself to ACC via a service call.
This registration timer runs asynchronously to the rest of the node's functionality.

Upon successful registration:

- The registration timer is stopped.
- ACC will create a subscription to the HB messages of AN.
- ACC will return a UUID specific to the AN instance.

### Error state

WIP.
Currently, only sends positive heartbeat messages.

### Heartbeat (**HB**)

Relevant parameters:

| Parameter             | Function                                                               |
| --------------------- | ---------------------------------------------------------------------- |
| `deadline_ms`         | If ACC doesn't receive a HB by this deadline, AN will be assumed dead. |
| `period_heartbeat_ms` | AN is expected to publish a HB with this period.                       |

- 🟡 `deadline_ms` should be slightly higher than `period_heartbeat_ms` to account for network delays.
- 🔴 `deadline_ms` of AN should match the `deadline_ms` of ACC.

Upon registration, AN starts a timer that periodically publishes HB messages to ACC.
This HB timer runs asynchronously to the rest of the node's functionality.

### Monitored subscription

WIP.

## Credits

- Heartbeat functionality is based on ROS 2 [software_watchdogs](https://github.com/ros-safety/software_watchdogs)
  package.
