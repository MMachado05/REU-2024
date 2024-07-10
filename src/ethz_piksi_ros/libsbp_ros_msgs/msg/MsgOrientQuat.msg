# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Orientation
# MSG_ORIENT_QUAT 0x0220
#
# Quaternion 4 component vector
#
# This message reports the quaternion vector describing the vehicle body frame's orientation
# with respect to a local-level NED frame. The components of the vector should sum to a unit
# vector assuming that the LSB of each component as a value of 2^-31. This message will only
# be available in future INS versions of Swift Products and is not produced by Piksi Multi 
# or Duro.

# GPS Time of Week
uint32 tow # [ms]

# Real component
int32 w # [2^-31]

# 1st imaginary component
int32 x # [2^-31]

# 2nd imaginary component
int32 y # [2^-31]

# 3rd imaginary component
int32 z # [2^-31]

# Estimated standard deviation of w
float32 w_accuracy # [N/A]

# Estimated standard deviation of x
float32 x_accuracy # [N/A]

# Estimated standard deviation of y
float32 y_accuracy # [N/A]

# Estimated standard deviation of z
float32 z_accuracy # [N/A]

# Status flags
uint8 flags # []
