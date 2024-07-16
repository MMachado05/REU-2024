# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Imu
# MSG_IMU_RAW 0x0900
#
# Raw IMU data
#
# Raw data from the Inertial Measurement Unit, containing accelerometer and
# gyroscope readings. The sense of the measurements are to be aligned with 
# the indications on the device itself. Measurement units, which are specific to the
# device hardware and settings, are communicated via the MSG_IMU_AUX message.

# Milliseconds since start of GPS week. If the high bit is set, the
# time is unknown or invalid.
uint32 tow # [ms]

# Milliseconds since start of GPS week, fractional part
uint8 tow_f # [ms / 256]

# Acceleration in the IMU frame X axis
int16 acc_x # []

# Acceleration in the IMU frame Y axis
int16 acc_y # []

# Acceleration in the IMU frame Z axis
int16 acc_z # []

# Angular rate around IMU frame X axis
int16 gyr_x # []

# Angular rate around IMU frame Y axis
int16 gyr_y # []

# Angular rate around IMU frame Z axis
int16 gyr_z # []
