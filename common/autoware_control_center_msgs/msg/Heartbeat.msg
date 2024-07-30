# Heartbeat message designed to monitor node liveliness with an additional
# sequence number to track message order and detect potential drops.

# Stamp when the message was generated
builtin_interfaces/Time stamp

# Node identifier
unique_identifier_msgs/UUID uuid_node

# Incremental sequence number used to verify the order of received heartbeats
# and identify potential message drops or delays.
# This number will wrap around once it reaches to its maximum value (65535)
uint16 sequence_number

# Status of the node
autoware_control_center_msgs/NodeStatusActivity status_activity
autoware_control_center_msgs/NodeStatusOperational status_operational
