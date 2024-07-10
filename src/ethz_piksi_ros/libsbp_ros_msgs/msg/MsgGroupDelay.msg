# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_GROUP_DELAY 0x0094
#
# Group Delay
#
# Please see ICD-GPS-200 (30.3.3.3.1.1) for more details.

# Data Predict Time of Week
GpsTimeSec t_op # []

# GNSS signal identifier
GnssSignal sid # []

# bit-field indicating validity of the values,
# LSB indicating tgd validity etc.
# 1 = value is valid, 0 = value is not valid.
uint8 valid # []


int16 tgd # [s * 2^-35]


int16 isc_l1ca # [s * 2^-35]


int16 isc_l2c # [s * 2^-35]
