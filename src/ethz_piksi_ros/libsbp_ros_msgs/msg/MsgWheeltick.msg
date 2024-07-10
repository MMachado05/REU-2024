# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Vehicle
# MSG_WHEELTICK 0x0904
#
# Accumulated wheeltick count message
#
# Message containing the accumulated distance travelled by a wheel located at an odometry
# reference point defined by the user. The offset for the odometry reference point and the
# definition and origin of the user frame are defined through the device settings interface.
# The source of this message is identified by the source field, which is an integer ranging
# from 0 to 255.
# The timestamp associated with this message should represent the time when the accumulated
# tick count reached the value given by the contents of this message as accurately as possible.

# Time field representing either microseconds since the last PPS, microseconds in the GPS
# Week or local CPU time from the producing system in microseconds. See the synch_type
# field for the exact meaning of this timestamp.
uint64 time # [us]

# Field indicating the type of timestamp contained in the time field.
uint8 flags # []

# ID of the sensor producing this message
uint8 source # []

# Free-running counter of the accumulated distance for this sensor. The counter should be
# incrementing if travelling into one direction and decrementing when travelling in the
# opposite direction.
int32 ticks # [arbitrary distance units]
