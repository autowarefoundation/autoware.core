# Heartbeat message designed to monitor node liveliness with an additional
# sequence number to track message order and detect potential drops.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Incremental sequence number used to verify the order of received heartbeats
# and identify potential message drops or delays.
# This number will wrap around once it reaches to its maximum value (65535)
uint16 sequence_number 0