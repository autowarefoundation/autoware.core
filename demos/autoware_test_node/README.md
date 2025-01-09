# autoware_test_node

This package contains a simple example of how to use `autoware::Node`.

## Usage

```bash
ros2 launch autoware_test_node autoware_test_node.launch.xml
```

### Lifecycle control

Information on Lifecycle nodes can be found [here](https://design.ros2.org/articles/node_lifecycle.html).

Output a list of nodes with lifecycle:

```console
$ ros2 lifecycle nodes
/test_ns1/test_node1
```

Get the current state of a node:

```console
$ ros2 lifecycle get /test_ns1/test_node1
unconfigured [1]
```

List the available transitions for the node:

```console
$ ros2 lifecycle list /test_ns1/test_node1
- configure [1]
 Start: unconfigured
 Goal: configuring
- shutdown [5]
 Start: unconfigured
 Goal: shuttingdown
```

Shutdown the node:

```console
$ ros2 lifecycle set /test_ns1/test_node1 shutdown
Transitioning successful
```

```console
$ ros2 lifecycle get /test_ns1/test_node1
finalized [4]
```

The node will remain alive in the `finalized` state until it is killed by the user.
