# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Tracking
# MSG_TRACKING_IQ 0x002D
#
# Tracking channel correlations
#
# When enabled, a tracking channel can output the correlations at each
# update interval.

# Tracking channel of origin
uint8 channel # []

# GNSS signal identifier
GnssSignal sid # []

# Early, Prompt and Late correlations
TrackingChannelCorrelation[] corrs # []
