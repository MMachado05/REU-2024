# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# MSG_MASK_SATELLITE 0x002B
#
# Mask a satellite from use in Piksi subsystems
#
# This message allows setting a mask to prevent a particular satellite
# from being used in various Piksi subsystems.

# Mask of systems that should ignore this satellite.
uint8 mask # []

# GNSS signal for which the mask is applied
GnssSignal sid # []
