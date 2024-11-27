# autoware_test_node

This package contains a simple example of how to use `autoware::Node`.

## Usage

```bash
ros2 launch autoware_test_node autoware_node_with_cc.launch.xml
```

On another terminal:

```bash
ros2 topic echo /autoware/control_center/node_reports
```

You should see the reports coming from the nodes.

### Lifecycle control

Output a list of nodes with lifecycle:

```console
$ ros2 lifecycle nodes
/autoware/control_center
/test_node3
/test_node4
/test_ns1/test_node1
/test_ns2/test_node2
```

Get the current state of a node:

```console
$ ros2 lifecycle get /test_node4
unconfigured [1]
```

List the available transitions for the node:

```console
$ ros2 lifecycle list /test_node4
- configure [1]
 Start: unconfigured
 Goal: configuring
- shutdown [5]
 Start: unconfigured
 Goal: shuttingdown
```

Shutdown the node:

```console
$ ros2 lifecycle set /test_node4 shutdown
Transitioning successful
```

Check the `/autoware/control_center/node_reports` topic to see the node's `is_alive` field change to `false`.
