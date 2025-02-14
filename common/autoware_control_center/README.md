# Autoware Control Center

## Abbreviations

- **ACC:** Autoware Control Center
- **AN:** Autoware Node

## Overview

The Autoware Control Center (ACC) is a core package within the Autoware system, designed to manage and monitor Autoware
nodes (ANs).

It provides services for the registration, de-registration, of ANs, as well as publishing reports on
their status.

ACC maintains an internal registry to keep track of ANs.

## Interfaces

### Services

#### Register

Registers an Autoware node to the ACC.

- **Topic:** `/autoware/control_center/srv/register`
- **Type:** `autoware_control_center_msgs::srv::Register`

#### Deregister

De-registers an Autoware node from the ACC.

- **Topic:** `/autoware/control_center/srv/deregister`
- **Type:** `autoware_control_center_msgs::srv::Deregister`

### Publishers

#### NodeReports

Publishes reports on the current status of registered Autoware nodes.

- **Topic:** `/autoware/control_center/node_reports`
- **Type:** `autoware_control_center_msgs::msg::NodeReports`

## Parameters

| Name                  | Type     | Default Value | Description                                           |
| --------------------- | -------- | ------------- | ----------------------------------------------------- |
| `report_publish_rate` | `double` | `1.0`         | Frequency (in Hz) at which NodeReports are published. |

## Singleton Constraint

To ensure that only one instance of the ACC is running at any given time, two checks are performed in the main function:

### 1. Lockfile Check

This lockfile mechanism is fast and reliable for single-machine scenarios.
It prevents multiple instances of ACC from running concurrently on the same machine.

**Path:** `/tmp/autoware_control_center_node.lock`

### 2. Network-Wide Node Name Check

This involves gathering all node names and comparing them with the ACC node name (`/autoware/control_center`).
While this method is slower and less reliable than the lockfile check,
it is necessary for scenarios where the ACC is run across a network of machines.
This ensures that no other instance of ACC is running on any other machine within the network.

## Usage

`ros2 launch autoware_control_center control_center.launch.xml`

## Workflow

When an Autoware Node starts, it registers itself with the ACC.
