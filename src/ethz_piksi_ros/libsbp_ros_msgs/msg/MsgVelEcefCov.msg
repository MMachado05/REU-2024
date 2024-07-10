# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_VEL_ECEF_COV 0x0215
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

# Estimated variance of x
float32 cov_x_x # [m^2/s^2]

# Estimated covariance of x and y
float32 cov_x_y # [m^2/s^2]

# Estimated covariance of x and z
float32 cov_x_z # [m^2/s^2]

# Estimated variance of y
float32 cov_y_y # [m^2/s^2]

# Estimated covariance of y and z
float32 cov_y_z # [m^2/s^2]

# Estimated variance of z
float32 cov_z_z # [m^2/s^2]

# Number of satellites used in solution
uint8 n_sats # []

# Status flags
uint8 flags # []
