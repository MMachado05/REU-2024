# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Orientation
# MSG_ORIENT_EULER 0x0221
#
# Euler angles
#
# This message reports the yaw, pitch, and roll angles of the vehicle body frame.
# The rotations should applied intrinsically in the order yaw, pitch, and roll 
# in order to rotate the from a frame aligned with the local-level NED frame 
# to the vehicle body frame.  This message will only be available in future 
# INS versions of Swift Products and is not produced by Piksi Multi or Duro.

# GPS Time of Week
uint32 tow # [ms]

# rotation about the forward axis of the vehicle
int32 roll # [microdegrees]

# rotation about the rightward axis of the vehicle
int32 pitch # [microdegrees]

# rotation about the downward axis of the vehicle
int32 yaw # [microdegrees]

# Estimated standard deviation of roll
float32 roll_accuracy # [degrees]

# Estimated standard deviation of pitch
float32 pitch_accuracy # [degrees]

# Estimated standard deviation of yaw
float32 yaw_accuracy # [degrees]

# Status flags
uint8 flags # []
