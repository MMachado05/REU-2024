# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_OBS 0x004A
#
# GPS satellite observations
#
# The GPS observations message reports all the raw pseudorange and
# carrier phase observations for the satellites being tracked by
# the device. Carrier phase observation here is represented as a
# 40-bit fixed point number with Q32.8 layout (i.e. 32-bits of
# whole cycles and 8-bits of fractional cycles). The observations
# are be interoperable with 3rd party receivers and conform
# with typical RTCMv3 GNSS observations.

# Header of a GPS observation message
ObservationHeader header # []

# Pseudorange and carrier phase observation for a
# satellite being tracked.
PackedObsContent[] obs # []
