# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_POS_ECEF_COV_GNSS 0x0234
#
# GNSS-only Position in ECEF
#
# The position solution message reports absolute Earth Centered
# Earth Fixed (ECEF) coordinates and the status (single point vs
# pseudo-absolute RTK) of the position solution. The message also
# reports the upper triangular portion of the 3x3 covariance matrix.
# If the receiver knows the surveyed position of the base station and has
# an RTK solution, this reports a pseudo-absolute position
# solution using the base station position and the rover's RTK
# baseline vector. The full GPS time is given by the preceding
# MSG_GPS_TIME with the matching time-of-week (tow).

# GPS Time of Week
uint32 tow # [ms]

# ECEF X coordinate
float64 x # [m]

# ECEF Y coordinate
float64 y # [m]

# ECEF Z coordinate
float64 z # [m]

# Estimated variance of x
float32 cov_x_x # [m^2]

# Estimated covariance of x and y
float32 cov_x_y # [m^2]

# Estimated covariance of x and z
float32 cov_x_z # [m^2]

# Estimated variance of y
float32 cov_y_y # [m^2]

# Estimated covariance of y and z
float32 cov_y_z # [m^2]

# Estimated variance of z
float32 cov_z_z # [m^2]

# Number of satellites used in solution
uint8 n_sats # []

# Status flags
uint8 flags # []
