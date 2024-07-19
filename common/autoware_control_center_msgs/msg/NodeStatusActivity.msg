# Enumeration of possible activity statuses for Autoware nodes.
uint8 UNKNOWN=0              # Activity status unknown
uint8 INITIALIZING=1         # Node is initializing
uint8 IDLE=2                 # Node is idle
uint8 PROCESSING=3           # Node is actively processing
uint8 WAITING_FOR_MESSAGE=4  # Node is waiting for a message
uint8 SHUTTING_DOWN=5        # Node is shutting down

# Current activity status
uint8 status
