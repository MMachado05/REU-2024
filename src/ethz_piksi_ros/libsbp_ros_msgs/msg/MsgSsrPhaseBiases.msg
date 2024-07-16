# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# MSG_SSR_PHASE_BIASES 0x05E6
#
# Precise phase biases correction
#
# The precise phase biases message contains the biases
# to be added to the carrier phase of the corresponding
# signal to get corrected carrier phase measurement, as
# well as the satellite yaw angle to be applied to compute
# the phase wind-up correction.
# It is typically an equivalent to the 1265 RTCM message types

# GNSS reference time of the correction
GpsTimeSec time # []

# GNSS signal identifier (16 bit)
GnssSignal sid # []

# Update interval between consecutive corrections. Encoded
# following RTCM DF391 specification.
uint8 update_interval # []

# IOD of the SSR correction. A change of Issue Of Data
# SSR is used to indicate a change in the SSR
# generating configuration
uint8 iod_ssr # []

# Indicator for the dispersive phase biases property.
uint8 dispersive_bias # []

# Consistency indicator for Melbourne-Wubbena linear combinations
uint8 mw_consistency # []

# Satellite yaw angle
uint16 yaw # [1 / 256 semi-circle]

# Satellite yaw angle rate
int8 yaw_rate # [1 / 8192 semi-circle / s]

# Phase biases corrections for a
# satellite being tracked.
PhaseBiasesContent[] biases # []
