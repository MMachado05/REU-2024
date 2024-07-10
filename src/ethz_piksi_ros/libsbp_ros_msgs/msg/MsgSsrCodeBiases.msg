# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# MSG_SSR_CODE_BIASES 0x05E1
#
# Precise code biases correction
#
# The precise code biases message is to be added
# to the pseudorange of the corresponding signal
# to get corrected pseudorange. It is typically
# an equivalent to the 1059 and 1065 RTCM message types

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

# Code biases for the different satellite signals
CodeBiasesContent[] biases # []
