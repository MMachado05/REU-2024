# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_BASELINE_ECEF 0x020B
#
# Baseline Position in ECEF
#
# This message reports the baseline solution in Earth Centered
# Earth Fixed (ECEF) coordinates. This baseline is the relative
# vector distance from the base station to the rover receiver. The
# full GPS time is given by the preceding MSG_GPS_TIME with the
# matching time-of-week (tow).

# GPS Time of Week
uint32 tow # [ms]

# Baseline ECEF X coordinate
int32 x # [mm]

# Baseline ECEF Y coordinate
int32 y # [mm]

# Baseline ECEF Z coordinate
int32 z # [mm]

# Position estimated standard deviation
uint16 accuracy # [mm]

# Number of satellites used in solution
uint8 n_sats # []

# Status flags
uint8 flags # []
