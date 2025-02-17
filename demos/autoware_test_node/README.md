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
