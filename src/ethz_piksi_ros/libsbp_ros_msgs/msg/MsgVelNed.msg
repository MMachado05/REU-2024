# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_VEL_NED 0x020E
#
# Velocity in NED
#
# This message reports the velocity in local North East Down (NED)
# coordinates. The NED coordinate system is defined as the local WGS84
# tangent plane centered at the current position. The full GPS time is
# given by the preceding MSG_GPS_TIME with the matching time-of-week (tow).

# GPS Time of Week
uint32 tow # [ms]

# Velocity North coordinate
int32 n # [mm/s]

# Velocity East coordinate
int32 e # [mm/s]

# Velocity Down coordinate
int32 d # [mm/s]

# Horizontal velocity estimated standard deviation
uint16 h_accuracy # [mm/s]

# Vertical velocity estimated standard deviation
uint16 v_accuracy # [mm/s]

# Number of satellites used in solution
uint8 n_sats # []

# Status flags
uint8 flags # []
