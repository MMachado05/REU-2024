# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# MSG_THREAD_STATE 0x0017
#
# State of an RTOS thread
#
# The thread usage message from the device reports real-time
# operating system (RTOS) thread usage statistics for the named
# thread. The reported percentage values must be normalized.

# Thread name (NULL terminated)
string name # []

# Percentage cpu use for this thread. Values range from 0
# - 1000 and needs to be renormalized to 100
uint16 cpu # []

# Free stack space for this thread
uint32 stack_free # [bytes]
