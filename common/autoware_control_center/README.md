# Autoware Control Center

## Overview

Autoware Control Center is a Autoware.Core package designed to manage and monitor Autoware nodes within a system. It provides services for registering, deregistering, and handling errors for Autoware nodes, as well as publishing reports on their status.

ACC capabilities include:

- ACC provides *AutowareNodeRegister* service. It allows each *autoware_node* to be registered to ACC instance. 
- ACC provides  *AutowareNodeDeregister* service. *Autoware_node* is able to deregister it self from ACC via call to this service.
- ACC keeps track of registered *autoware_nodes* internally with use of the *node_registry*.
- ACC provides *AutowareNodeError* service. It allows each *autoware_node* send it's state to ACC.
- ACC subscribe to *heartbeat* topic of *autoware_node* after it's registration. ACC controls liveliness of *autoware_node* by  monitoring this topic. 
- ACC publishes reports on a current status of registered *autoware_nodes* to the *autoware_node_reports* topic.

## Usage 

There is no dedicated launch file for autoware_control_center. So you need to run it with command:
```bash
ros2 run autoware_control_center autoware_control_center
```  

ACC has startup timer and waits for 10 sec for any node to be registered. If atleast one node is registered ACC starts normal work. If not ACC will think that it was relaunched after crash and will start re-register procedure.   

It will list all *autoware_nodes* with *AutowareContolCenterDeregister* service and will send request to each node. So all nodes will have to register to  the new instance of ACC. After this proceedure ACC will start regular work.  If the list will be empty ACC will keep going and will publish empty messages to the *autoware_node_reports* topic.

Expected heartbeat frequency is 5 Hz. It can be configured by `lease_duration` parameter. Lease duration must be >= heartbeat's period in *autoware_node* as there is some network overhead. If the granted `lease_duration`` time will be violated such *autoware_node* will be considered as not alive. 

## Parameters 

| Name              | Type | Default Value | Description                                                                              |
| ----------------- | ---- | ------------- | ---------------------------------------------------------------------------------------- |
| `lease_duration`  | int  | `220`         | After a violation this heartbeat period *autoware_node* will be considered as not alive  |

## Design 

Heartbeat funtionality is based on ros2 [software_watchdogs](https://github.com/ros-safety/software_watchdogs) package. 