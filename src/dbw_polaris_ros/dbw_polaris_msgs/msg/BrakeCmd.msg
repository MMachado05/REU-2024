# Brake pedal
# Options defined below
float32 pedal_cmd
uint8 pedal_cmd_type

# Enable
bool enable

# Clear driver overrides
bool clear

# Ignore driver overrides
bool ignore

# Watchdog counter (optional)
uint8 count

uint8 CMD_NONE=0
uint8 CMD_PERCENT=2   # Percent of maximum torque, range 0 to 1
uint8 CMD_TORQUE=3    # Nm, range 0 to 8000, open-loop
uint8 CMD_TORQUE_RQ=4 # Nm, range 0 to 8000, closed-loop

float32 TORQUE_MAX=8000 # Nm, maximum torque
