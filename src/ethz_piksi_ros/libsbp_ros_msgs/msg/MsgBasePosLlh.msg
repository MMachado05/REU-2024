# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_BASE_POS_LLH 0x0044
#
# Base station position
#
# The base station position message is the position reported by
# the base station itself. It is used for pseudo-absolute RTK
# positioning, and is required to be a high-accuracy surveyed
# location of the base station. Any error here will result in an
# error in the pseudo-absolute position output.

# Latitude
float64 lat # [deg]

# Longitude
float64 lon # [deg]

# Height
float64 height # [m]
