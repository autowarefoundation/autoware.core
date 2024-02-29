# NodeStatus message designed to provide current status of Autoware node registered to Autoware Control Center

unique_identifier_msgs/UUID uuid_node
string name_node
bool alive
autoware_control_center_msgs/AutowareNodeState last_node_state
string node_report
builtin_interfaces/Duration last_heartbeat
