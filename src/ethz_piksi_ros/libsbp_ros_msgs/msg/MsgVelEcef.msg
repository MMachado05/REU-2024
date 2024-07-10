# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_VEL_ECEF 0x020D
#
# Velocity in ECEF
#
# This message reports the velocity in Earth Centered Earth Fixed
# (ECEF) coordinates. The full GPS time is given by the preceding
# MSG_GPS_TIME with the matching time-of-week (tow).

# GPS Time of Week
uint32 tow # [ms]

# Velocity ECEF X coordinate
int32 x # [mm/s]

# Velocity ECEF Y coordinate
int32 y # [mm/s]

# Velocity ECEF Z coordinate
int32 z # [mm/s]

# Velocity estimated standard deviation
uint16 accuracy # [mm/s]

# Number of satellites used in solution
uint8 n_sats # []

# Status flags
uint8 flags # []
