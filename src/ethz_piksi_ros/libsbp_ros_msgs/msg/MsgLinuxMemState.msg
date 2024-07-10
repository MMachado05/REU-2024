# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Linux
# MSG_LINUX_MEM_STATE 0x7F01
#
# List CPU state on the system
#
# This message indicates the process state of the top 10 heaviest
# consumers of memory on the system.

# sequence of this status message, values from 0-9
uint8 index # []

# the PID of the process
uint16 pid # []

# percent of memory used, expressed as a fraction of 256
uint8 pmem # []

# fixed length string representing the thread name
string tname # []

# the command line (as much as it fits in the remaining packet)
string cmdline # []
