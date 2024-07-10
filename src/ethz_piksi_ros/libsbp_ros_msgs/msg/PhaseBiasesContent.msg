# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# PhaseBiasesContent
#
# SSR phase biases corrections for a particular satellite.
#
# Phase biases are to be added to carrier phase measurements.
# The corrections conform with typical RTCMv3 MT1059 and 1065.

# Signal constellation, band and code
uint8 code # []

# Indicator for integer property
uint8 integer_indicator # []

# Indicator for two groups of Wide-Lane(s) integer property
uint8 widelane_integer_indicator # []

# Signal phase discontinuity counter.
# Increased for every discontinuity in phase.
uint8 discontinuity_counter # []

# Phase bias for specified signal
int32 bias # [0.1 mm]
