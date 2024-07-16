# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Mag
# MSG_MAG_RAW 0x0902
#
# Raw magnetometer data
#
# Raw data from the magnetometer.

# Milliseconds since start of GPS week. If the high bit is set, the
# time is unknown or invalid.
uint32 tow # [ms]

# Milliseconds since start of GPS week, fractional part
uint8 tow_f # [ms / 256]

# Magnetic field in the body frame X axis
int16 mag_x # [microteslas]

# Magnetic field in the body frame Y axis
int16 mag_y # [microteslas]

# Magnetic field in the body frame Z axis
int16 mag_z # [microteslas]
