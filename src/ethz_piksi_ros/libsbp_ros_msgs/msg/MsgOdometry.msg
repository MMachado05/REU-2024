# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Vehicle
# MSG_ODOMETRY 0x0903
#
# Vehicle forward (x-axis) velocity
#
# Message representing the x component of vehicle velocity in the user frame at the odometry
# reference point(s) specified by the user. The offset for the odometry reference point and 
# the definition and origin of the user frame are defined through the device settings interface.
# There are 4 possible user-defined sources of this message  which are labeled arbitrarily 
# source 0 through 3.

# Time field representing either milliseconds in the GPS Week or local CPU
# time from the producing system in milliseconds.  See the tow_source flag
# for the exact source of this timestamp.
uint32 tow # [ms]

# The signed forward component of vehicle velocity.
int32 velocity # [mm/s]

# Status flags
uint8 flags # []
