# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# EphemerisCommonContent
#
# Common fields for every ephemeris message.
#


# GNSS signal identifier (16 bit)
GnssSignal sid # []

# Time of Ephemerides
GpsTimeSec toe # []

# User Range Accuracy
float32 ura # [m]

# Curve fit interval
uint32 fit_interval # [s]

# Status of ephemeris, 1 = valid, 0 = invalid
uint8 valid # []

# Satellite health status.
# GPS: ICD-GPS-200, chapter 20.3.3.3.1.4
# SBAS: 0 = valid, non-zero = invalid
# GLO: 0 = valid, non-zero = invalid
uint8 health_bits # []
