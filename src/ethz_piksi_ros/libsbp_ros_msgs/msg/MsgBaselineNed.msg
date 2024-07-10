# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_BASELINE_NED 0x020C
#
# Baseline in NED
#
# This message reports the baseline solution in North East Down
# (NED) coordinates. This baseline is the relative vector distance
# from the base station to the rover receiver, and NED coordinate
# system is defined at the local WGS84 tangent plane centered at the
# base station position.  The full GPS time is given by the
# preceding MSG_GPS_TIME with the matching time-of-week (tow).

# GPS Time of Week
uint32 tow # [ms]

# Baseline North coordinate
int32 n # [mm]

# Baseline East coordinate
int32 e # [mm]

# Baseline Down coordinate
int32 d # [mm]

# Horizontal position estimated standard deviation
uint16 h_accuracy # [mm]

# Vertical position estimated standard deviation
uint16 v_accuracy # [mm]

# Number of satellites used in solution
uint8 n_sats # []

# Status flags
uint8 flags # []
