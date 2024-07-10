# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Orientation
# MSG_ANGULAR_RATE 0x0222
#
# Vehicle Body Frame instantaneous angular rates
#
# This message reports the orientation rates in the vehicle body frame. 
# The values represent the measurements a strapped down gyroscope would 
# make and are not equivalent to the time derivative of the Euler angles.
# The orientation and origin of the user frame is specified via device settings.
# By convention, the vehicle x-axis is expected to be aligned with the forward
# direction, while the vehicle y-axis is expected to be aligned with the right
# direction, and the vehicle z-axis should be aligned with the down direction.
# This message will only be available in future INS versions of Swift Products 
# and is not produced by Piksi Multi or Duro.

# GPS Time of Week
uint32 tow # [ms]

# angular rate about x axis
int32 x # [microdegrees/s]

# angular rate about y axis
int32 y # [microdegrees/s]

# angular rate about z axis
int32 z # [microdegrees/s]

# Status flags
uint8 flags # []
