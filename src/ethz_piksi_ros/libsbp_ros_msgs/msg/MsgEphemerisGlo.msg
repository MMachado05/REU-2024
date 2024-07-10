# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_EPHEMERIS_GLO 0x008B
#
# Satellite broadcast ephemeris for GLO
#
# The ephemeris message returns a set of satellite orbit
# parameters that is used to calculate GLO satellite position,
# velocity, and clock offset. Please see the GLO ICD 5.1 "Table 4.5
# Characteristics of words of immediate information (ephemeris parameters)"
# for more details.

# Values common for all ephemeris types
EphemerisCommonContent common # []

# Relative deviation of predicted carrier frequency from nominal
float32 gamma # []

# Correction to the SV time
float32 tau # [s]

# Equipment delay between L1 and L2
float32 d_tau # [s]

# Position of the SV at tb in PZ-90.02 coordinates system
float64[] pos # [m]

# Velocity vector of the SV at tb in PZ-90.02 coordinates system
float64[] vel # [m/s]

# Acceleration vector of the SV at tb in PZ-90.02 coordinates sys
float32[] acc # [m/s^2]

# Frequency slot. FCN+8 (that is [1..14]). 0 or 0xFF for invalid
uint8 fcn # []

# Issue of ephemeris data
uint8 iod # []
