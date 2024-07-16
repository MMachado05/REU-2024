# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_PROTECTION_LEVEL 0x0216
#
# Computed Position and Protection Level
#
# This message reports the local vertical and horizontal protection levels
# associated with a given LLH position solution. The full GPS time is given
# by the preceding MSG_GPS_TIME with the matching time-of-week (tow).

# GPS Time of Week
uint32 tow # [ms]

# Vertical protection level
uint16 vpl # [cm]

# Horizontal protection level
uint16 hpl # [cm]

# Latitude
float64 lat # [deg]

# Longitude
float64 lon # [deg]

# Height
float64 height # [m]

# Status flags
uint8 flags # []
