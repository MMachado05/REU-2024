# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Linux
# MSG_LINUX_SOCKET_USAGE 0x7F05
#
# Summary of socket usage across the system
#
# Summaries the socket usage across the system.

# average socket queue depths across all sockets on the system
uint32 avg_queue_depth # []

# the max queue depth seen within the reporting period
uint32 max_queue_depth # []

# A count for each socket type reported in the `socket_types_reported` field,
# the first entry corresponds to the first enabled bit in `types_reported`.
uint16[] socket_state_counts # []

# A count for each socket type reported in the `socket_types_reported` field,
# the first entry corresponds to the first enabled bit in `types_reported`.
uint16[] socket_type_counts # []
