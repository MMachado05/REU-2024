# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ext_Events
# MSG_EXT_EVENT 0x0101
#
# Reports timestamped external pin event
#
# Reports detection of an external event, the GPS time it occurred,
# which pin it was and whether it was rising or falling.

# GPS week number
uint16 wn # [weeks]

# GPS time of week rounded to the nearest millisecond
uint32 tow # [ms]

# Nanosecond residual of millisecond-rounded TOW (ranges
# from -500000 to 500000)
int32 ns_residual # [ns]

# Flags
uint8 flags # []

# Pin number.  0..9 = DEBUG0..9.
uint8 pin # []
