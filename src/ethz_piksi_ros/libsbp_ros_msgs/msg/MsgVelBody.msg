# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_VEL_BODY 0x0213
#
# Velocity in User Frame
#
# This message reports the velocity in the Vehicle Body Frame. By convention,
# the x-axis should point out the nose of the vehicle and represent the forward
# direction, while as the y-axis should point out the right hand side of the vehicle.
# Since this is a right handed system, z should point out the bottom of the vehicle.
# The orientation and origin of the Vehicle Body Frame are specified via the device settings.
# The full GPS time is given by the preceding MSG_GPS_TIME with the
# matching time-of-week (tow). This message is only produced by inertial versions of Swift
# products and is not available from Piksi Multi or Duro.

# GPS Time of Week
uint32 tow # [ms]

# Velocity in x direction
int32 x # [mm/s]

# Velocity in y direction
int32 y # [mm/s]

# Velocity in z direction
int32 z # [mm/s]

# Estimated variance of x
float32 cov_x_x # [m^2]

# Covariance of x and y
float32 cov_x_y # [m^2]

# Covariance of x and z
float32 cov_x_z # [m^2]

# Estimated variance of y
float32 cov_y_y # [m^2]

# Covariance of y and z
float32 cov_y_z # [m^2]

# Estimated variance of z
float32 cov_z_z # [m^2]

# Number of satellites used in solution
uint8 n_sats # []

# Status flags
uint8 flags # []
