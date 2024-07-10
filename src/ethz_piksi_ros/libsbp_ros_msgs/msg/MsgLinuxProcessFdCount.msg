# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Linux
# MSG_LINUX_PROCESS_FD_COUNT 0x7F06
#
# Summary of processes with large amounts of open file descriptors
#
# Top 10 list of processes with a large number of open file descriptors.

# sequence of this status message, values from 0-9
uint8 index # []

# the PID of the process in question
uint16 pid # []

# a count of the number of file descriptors opened by the process
uint16 fd_count # []

# the command line of the process in question
string cmdline # []
