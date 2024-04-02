# Autoware Control Center

## Overview

Autoware Control Center (ACC) is an Autoware.Core package designed to manage and monitor Autoware nodes within a system. It provides services for registration, de-registration and handling errors for Autoware nodes, as well as publishing reports on their status.

ACC capabilities include:

- ACC provides _AutowareNodeRegister_ service. It allows each _autoware_node_ to be registered to ACC instance.
- ACC provides _AutowareNodeDeregister_ service. _Autoware_node_ is able to deregister it self from ACC via call to this service.
- ACC keeps track of registered _autoware_nodes_ internally with use of the _node_registry_.
- ACC provides _AutowareNodeError_ service. It allows each _autoware_node_ send it's state to ACC.
- ACC subscribe to _heartbeat_ topic of _autoware_node_ after it's registration. ACC controls liveliness of _autoware_node_ by  monitoring this topic.
- ACC publishes reports on a current status of registered _autoware_nodes_ to the _autoware_node_reports_ topic.

## Usage

There is no dedicated launch file for autoware_control_center. So you need to run it with command:

```bash
ros2 run autoware_control_center autoware_control_center
```

ACC has startup timer and waits for 10 sec for any node to be registered. If at least one node is registered ACC starts normal work. If not ACC will think that it was relaunched after crash and will start re-register procedure.

It will list all _autoware_nodes_ with _AutowareControlCenterDeregister_ service and will send request to each node. So all nodes will have to register to the new instance of ACC. After this procedure ACC will start regular work. If the list will be empty ACC will keep going and will publish empty messages to the _autoware_node_reports_ topic.

Expected heartbeat frequency is 5 Hz. It can be configured by `lease_duration` parameter. Lease duration must be >= heartbeat's period in _autoware_node_ as there is some network overhead. If the granted `lease_duration` time will be violated such _autoware_node_ will be considered as not alive.

## Parameters

| Name                      | Type   | Default Value | Description                                                                                      |
| ------------------------- | ------ | ------------- | ------------------------------------------------------------------------------------------------ |
| `lease_duration`          | int    | `220`         | After a violation this heartbeat period (in ms) _autoware_node_ will be considered as not alive. |
| `startup_duration`        | double | `10.0`        | A period (in s) of ACC startup procedure.                                                        |
| `startup_callback_period` | int    | `500`         | A period (in ms) of ACC startup timer.                                                           |
| `node_report_period`      | int    | `1000`        | A period (in ms) of publishing to the autoware_node_reports topic.                               |

## Design

Heartbeat functionality is based on ros2 [software_watchdogs](https://github.com/ros-safety/software_watchdogs) package.
