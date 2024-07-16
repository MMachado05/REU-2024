# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_POS_LLH 0x020A
#
# Geodetic Position
#
# This position solution message reports the absolute geodetic
# coordinates and the status (single point vs pseudo-absolute RTK)
# of the position solution. If the rover receiver knows the
# surveyed position of the base station and has an RTK solution,
# this reports a pseudo-absolute position solution using the base
# station position and the rover's RTK baseline vector. The full
# GPS time is given by the preceding MSG_GPS_TIME with the
# matching time-of-week (tow).

# GPS Time of Week
uint32 tow # [ms]

# Latitude
float64 lat # [deg]

# Longitude
float64 lon # [deg]

# Height above WGS84 ellipsoid
float64 height # [m]

# Horizontal position estimated standard deviation
uint16 h_accuracy # [mm]

# Vertical position estimated standard deviation
uint16 v_accuracy # [mm]

# Number of satellites used in solution.
uint8 n_sats # []

# Status flags
uint8 flags # []
