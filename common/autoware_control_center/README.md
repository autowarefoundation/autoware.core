# Autoware Control Center

## Abbreviations

- **ACC:** Autoware Control Center
- **AN:** Autoware Node

## Overview

The Autoware Control Center (ACC) is a core package within the Autoware system, designed to manage and monitor Autoware nodes (ANs).
It provides services for the registration, de-registration, and error handling of ANs, as well as publishing reports on their status.
ACC maintains an internal registry to keep track of registered ANs.

## Services

### Register

Registers an Autoware node to the ACC.

- **Topic:** `/autoware/control_center/srv/register`
- **Type:** `autoware_control_center_msgs::srv::Register`

### Deregister

De-registers an Autoware node from the ACC.

- **Topic:** `/autoware/control_center/srv/deregister`
- **Type:** `autoware_control_center_msgs::srv::Deregister`

### ReportState

Reports the state of an Autoware node to the ACC.

- **Topic:** `/autoware/control_center/srv/report_state`
- **Type:** `autoware_control_center_msgs::srv::ReportState`

### Topics

- ACC subscribes to _heartbeat_ topic of _Autoware_Node_ after its registration. ACC controls liveliness' of
  _Autoware_Nodes_ by monitoring this topic.
- ACC publishes reports on a current status of registered _Autoware_Nodes_ to the _autoware_node_reports_ topic.

## Usage

There is no dedicated launch file for autoware_control_center. So you need to run it with command:

```bash
ros2 run autoware_control_center autoware_control_center
```

ACC has startup timer and waits for 10 sec for any node to be registered. If at least one node is registered ACC starts
normal work. If not ACC will think that it was relaunched after crash and will start re-register procedure.

It will list all _autoware_nodes_ with _ControlCenterDeregister_ service and will send request to each node. So all
nodes will have to register to the new instance of ACC. After this procedure ACC will start regular work. If the list
will be empty ACC will keep going and will publish empty messages to the _autoware_node_reports_ topic.

Expected heartbeat frequency is 5 Hz. It can be configured by `lease_duration` parameter. Lease duration must be >=
heartbeat's period in _autoware_node_ as there is some network overhead. If the granted `lease_duration` time will be
violated such _autoware_node_ will be considered as not alive.

## Parameters

| Name                      | Type   | Default Value | Description                                                                                      |
| ------------------------- | ------ | ------------- | ------------------------------------------------------------------------------------------------ |
| `lease_duration`          | int    | `220`         | After a violation this heartbeat period (in ms) _autoware_node_ will be considered as not alive. |
| `startup_duration`        | double | `10.0`        | A period (in s) of ACC startup procedure.                                                        |
| `startup_callback_period` | int    | `500`         | A period (in ms) of ACC startup timer.                                                           |
| `node_report_period`      | int    | `1000`        | A period (in ms) of publishing to the autoware_node_reports topic.                               |

## Design

Heartbeat functionality is based on ros2 [software_watchdogs](https://github.com/ros-safety/software_watchdogs) package.
