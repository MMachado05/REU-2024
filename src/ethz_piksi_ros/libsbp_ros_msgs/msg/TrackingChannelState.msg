# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Tracking
# TrackingChannelState
#
# Signal tracking channel state
#
# Tracking channel state for a specific satellite signal and
# measured signal power.

# GNSS signal being tracked
GnssSignal sid # []

# Frequency channel number (GLONASS only)
uint8 fcn # []

# Carrier-to-Noise density.  Zero implies invalid cn0.
uint8 cn0 # [dB Hz / 4]
