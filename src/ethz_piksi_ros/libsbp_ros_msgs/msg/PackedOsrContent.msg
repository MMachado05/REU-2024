# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# PackedOsrContent
#
# Network correction for a particular satellite signal.
#
# Pseudorange and carrier phase network corrections for a satellite signal.

# Pseudorange observation
uint32 P # [2 cm]

# Carrier phase observation with typical sign convention.
CarrierPhase L # [cycles]

# Lock timer. This value gives an indication of the time
# for which a signal has maintained continuous phase lock.
# Whenever a signal has lost and regained lock, this
# value is reset to zero. It is encoded according to DF402 from
# the RTCM 10403.2 Amendment 2 specification.  Valid values range
# from 0 to 15 and the most significant nibble is reserved for future use.
uint8 lock # []

# Correction flags.
uint8 flags # []

# GNSS signal identifier (16 bit)
GnssSignal sid # []

# Slant ionospheric correction standard deviation
uint16 iono_std # [5 mm]

# Slant tropospheric correction standard deviation
uint16 tropo_std # [5 mm]

# Orbit/clock/bias correction projected on range standard deviation
uint16 range_std # [5 mm]
