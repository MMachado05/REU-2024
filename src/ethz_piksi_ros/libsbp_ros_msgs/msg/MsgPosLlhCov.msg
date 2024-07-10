# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_POS_LLH_COV 0x0211
#
# Geodetic Position
#
# This position solution message reports the absolute geodetic
# coordinates and the status (single point vs pseudo-absolute RTK)
# of the position solution as well as the upper triangle of the 3x3
# covariance matrix.  The position information and Fix Mode flags should
# follow the MSG_POS_LLH message.  Since the covariance matrix is computed
# in the local-level North, East, Down frame, the covariance terms follow
# with that convention. Thus, covariances are reported against the "downward"
# measurement and care should be taken with the sign convention.

# GPS Time of Week
uint32 tow # [ms]

# Latitude
float64 lat # [deg]

# Longitude
float64 lon # [deg]

# Height above WGS84 ellipsoid
float64 height # [m]

# Estimated variance of northing
float32 cov_n_n # [m^2]

# Covariance of northing and easting
float32 cov_n_e # [m^2]

# Covariance of northing and downward measurement
float32 cov_n_d # [m^2]

# Estimated variance of easting
float32 cov_e_e # [m^2]

# Covariance of easting and downward measurement
float32 cov_e_d # [m^2]

# Estimated variance of downward measurement
float32 cov_d_d # [m^2]

# Number of satellites used in solution.
uint8 n_sats # []

# Status flags
uint8 flags # []
