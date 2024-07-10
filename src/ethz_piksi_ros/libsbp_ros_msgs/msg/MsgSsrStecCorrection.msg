# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# MSG_SSR_STEC_CORRECTION 0x05EB
#
# Slant Total Electron Content
#
# The STEC per space vehicle, given as polynomial approximation for
# a given grid.  This should be combined with MSG_SSR_GRIDDED_CORRECTION
# message to get the state space representation of the atmospheric
# delay. It is typically equivalent to the QZSS CLAS Sub Type 8 messages

# Header of a STEC message
StecHeader header # []

# Array of STEC information for each space vehicle
StecSatElement[] stec_sat_list # []
