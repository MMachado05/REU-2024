# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Orientation
# MSG_BASELINE_HEADING 0x020F
#
# Heading relative to True North
#
# This message reports the baseline heading pointing from the base station
# to the rover relative to True North. The full GPS time is given by the
# preceding MSG_GPS_TIME with the matching time-of-week (tow). It is intended
# that time-matched RTK mode is used when the base station is moving.

# GPS Time of Week
uint32 tow # [ms]

# Heading
uint32 heading # [mdeg]

# Number of satellites used in solution
uint8 n_sats # []

# Status flags
uint8 flags # []
