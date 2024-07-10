# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Gnss
# GnssSignal
#
# Represents all the relevant information about the signal
#
# Signal identifier containing constellation, band, and satellite identifier

# Constellation-specific satellite identifier. This field for Glonass can  
# either be (100+FCN) where FCN is in [-7,+6] or 
# the Slot ID in [1,28]
uint8 sat # []

# Signal constellation, band and code
uint8 code # []
