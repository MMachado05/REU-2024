# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_BASE_POS_ECEF 0x0048
#
# Base station position in ECEF
#
# The base station position message is the position reported by
# the base station itself in absolute Earth Centered Earth Fixed
# coordinates. It is used for pseudo-absolute RTK positioning, and
# is required to be a high-accuracy surveyed location of the base
# station. Any error here will result in an error in the
# pseudo-absolute position output.

# ECEF X coodinate
float64 x # [m]

# ECEF Y coordinate
float64 y # [m]

# ECEF Z coordinate
float64 z # [m]
