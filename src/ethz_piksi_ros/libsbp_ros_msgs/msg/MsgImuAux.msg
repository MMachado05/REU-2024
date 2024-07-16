# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Imu
# MSG_IMU_AUX 0x0901
#
# Auxiliary IMU data
#
# Auxiliary data specific to a particular IMU. The `imu_type` field will
# always be consistent but the rest of the payload is device specific and
# depends on the value of `imu_type`.

# IMU type
uint8 imu_type # []

# Raw IMU temperature
int16 temp # []

# IMU configuration
uint8 imu_conf # []
