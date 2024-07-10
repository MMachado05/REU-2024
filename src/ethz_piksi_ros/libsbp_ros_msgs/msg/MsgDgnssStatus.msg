# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# System
# MSG_DGNSS_STATUS 0xFF02
#
# Status of received corrections
#
# This message provides information about the receipt of Differential
# corrections.  It is expected to be sent with each receipt of a complete
# corrections packet.

# Status flags
uint8 flags # []

# Latency of observation receipt
uint16 latency # [deci-seconds]

# Number of signals from base station
uint8 num_signals # []

# Corrections source string
string source # []
