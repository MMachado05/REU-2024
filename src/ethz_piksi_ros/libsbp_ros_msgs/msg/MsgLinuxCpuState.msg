# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Linux
# MSG_LINUX_CPU_STATE 0x7F00
#
# List CPU state on the system
#
# This message indicates the process state of the top 10 heaviest
# consumers of CPU on the system.

# sequence of this status message, values from 0-9
uint8 index # []

# the PID of the process
uint16 pid # []

# percent of cpu used, expressed as a fraction of 256
uint8 pcpu # []

# fixed length string representing the thread name
string tname # []

# the command line (as much as it fits in the remaining packet)
string cmdline # []
