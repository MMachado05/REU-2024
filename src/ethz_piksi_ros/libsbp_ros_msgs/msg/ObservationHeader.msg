# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# ObservationHeader
#
# Header for observation message.
#
# Header of a GNSS observation message.

# GNSS time of this observation
GpsTime t # []

# Total number of observations. First nibble is the size
# of the sequence (n), second nibble is the zero-indexed
# counter (ith packet of n)
uint8 n_obs # []
