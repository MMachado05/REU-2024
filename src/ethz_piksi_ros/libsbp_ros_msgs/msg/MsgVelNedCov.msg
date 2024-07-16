# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_VEL_NED_COV 0x0212
#
# Velocity in NED
#
# This message reports the velocity in local North East Down (NED)
# coordinates. The NED coordinate system is defined as the local WGS84
# tangent plane centered at the current position. The full GPS time is
# given by the preceding MSG_GPS_TIME with the matching time-of-week (tow).
# This message is similar to the MSG_VEL_NED, but it includes the upper triangular
# portion of the 3x3 covariance matrix.

# GPS Time of Week
uint32 tow # [ms]

# Velocity North coordinate
int32 n # [mm/s]

# Velocity East coordinate
int32 e # [mm/s]

# Velocity Down coordinate
int32 d # [mm/s]

# Estimated variance of northward measurement
float32 cov_n_n # [m^2]

# Covariance of northward and eastward measurement
float32 cov_n_e # [m^2]

# Covariance of northward and downward measurement
float32 cov_n_d # [m^2]

# Estimated variance of eastward measurement
float32 cov_e_e # [m^2]

# Covariance of eastward and downward measurement
float32 cov_e_d # [m^2]

# Estimated variance of downward measurement
float32 cov_d_d # [m^2]

# Number of satellites used in solution
uint8 n_sats # []

# Status flags
uint8 flags # []
