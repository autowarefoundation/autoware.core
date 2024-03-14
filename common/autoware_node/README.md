# Autoware Node 

## Overview

Autoware Node is an Autoware.Core package designed to provide a base class for all future nodes in the system. It provides ability to registrate node to *Autoware_control_center* (ACC), report node state, publish heartbeat and subscribe to monitored topics. It also inheritates all lifecycle control capabilities of the base class [LifecycleNode](https://docs.ros2.org/latest/api/rclcpp_lifecycle/classrclcpp__lifecycle_1_1LifecycleNode.html) 

## Usage

You can use *autoware_node* as a base class for any node in Autoware.Core system. There is an example package *test_node* which shows how *autoware_node* commuticate with ACC and other *autoware_nodes*. You can check it for more information. 

## Design 

*Autoware_node* inheritates from  ROS2 [*lifecycle_node*](https://design.ros2.org/articles/node_lifecycle.html) and has all basic fuctions of it. 

Below are the main add-ons and how they work.

### Registration

After startup each *autoware_node* tries to register itself to ACC via a service call of *AutowareNodeRegister*. It happens in  the dedicated timer. The timer will stop after the successful registration. 

### De-registraion 

If *autoware_node* receives a request to it's *AutowareControlCenterDeregister* service. It will disable the flag *registered* and it will startup timer which control registration client.

### Error state

*Autoware_node* has `send_state` method to send it's state to ACC via *AutowareNodeError* service. You need to provide `AutowareNodeState` and log message as parameters to the method. 

### Heartbeat

The heartbeat publisher is configured to publish the heartbeat message each 200 ms. You can change it by the `period` parameter during the launch of *autoware_node*. Be aware that you will also need to configure ACC accordingly. 

Heartbeat functionality is based on ros2 [software_watchdogs](https://github.com/ros-safety/software_watchdogs) package.

### Monitored subscription

*Autoware_node* provides the `create_monitored_subscription` method. It wraps around a standard `create_subscription` method  and adds a function to monitor a frequency of messages received in the topic. If it violates the condition provided in the `hz` parameter of the method *autoware_node* will send an error state to the ACC. 
 