# Periodic report from the Autoware Control Center on the status of a registered node.

# Node information
string name
unique_identifier_msgs/UUID uuid

# Registration information
builtin_interfaces/Time stamp_registration
uint16 count_registered

# Status of the node
autoware_control_center_msgs/NodeStatusActivity status_activity
autoware_control_center_msgs/NodeStatusOperational status_operational
builtin_interfaces/Time stamp_last_heartbeat
bool is_alive
