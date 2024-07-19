# Enumeration of possible operational statuses for Autoware nodes.
uint8 UNKNOWN=0  # Operational status unknown
uint8 NORMAL=1   # Node is operating normally
uint8 WARNING=2  # Node is in a warning status
uint8 ERROR=3    # Node has encountered an error
uint8 FATAL=4    # Node is in a fatal status

# Current operational status
uint8 status
