# Door command
uint8 UNKNOWN = 0 # Default or unknown value

uint8 IDLE = 1  # Commanded to be unpowered
uint8 OPEN = 2  # Commanded to open
uint8 CLOSE = 3 # Commanded to close

uint8 LOCK = 8         # Command to lock
uint8 UNLOCK = 9       # Command to unlock
uint8 NO_COMMAND = 10  # No command
uint8 TOGGLE = 11      # Toggle door lockedness

string name
uint8 position  # Select position from IDLE, OPEN, and CLOSE
uint8 lock  # Select lock from LOCK, UNLOCK, NO_COMMAND, and TOGGLE