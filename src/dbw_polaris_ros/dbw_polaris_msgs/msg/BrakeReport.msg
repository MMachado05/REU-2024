Header header

# Braking torque (Nm)
float32 torque_input
float32 torque_cmd
float32 torque_output

# Status
bool enabled  # Enabled
bool override # Driver override
bool driver   # Driver activity
bool timeout  # Command timeout

# Watchdog Counter
WatchdogCounter watchdog_counter
bool watchdog_braking
bool fault_wdc

# Faults
bool fault_ch1
bool fault_ch2
bool fault_power
