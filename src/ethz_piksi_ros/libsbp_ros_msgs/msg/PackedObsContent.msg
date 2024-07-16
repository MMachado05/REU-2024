# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# PackedObsContent
#
# GNSS observations for a particular satellite signal.
#
# Pseudorange and carrier phase observation for a satellite being tracked.
# The observations are interoperable with 3rd party receivers and conform with
# typical RTCM 3.1 message GPS/GLO observations.
# 
# Carrier phase observations are not guaranteed to be aligned to the RINEX 3
# or RTCM 3.3 MSM reference signal and no 1/4 cycle adjustments are currently
# peformed.

# Pseudorange observation
uint32 P # [2 cm]

# Carrier phase observation with typical sign convention.
CarrierPhase L # [cycles]

# Doppler observation with typical sign convention.
Doppler D # [Hz]

# Carrier-to-Noise density.  Zero implies invalid cn0.
uint8 cn0 # [dB Hz / 4]

# Lock timer. This value gives an indication of the time
# for which a signal has maintained continuous phase lock.
# Whenever a signal has lost and regained lock, this
# value is reset to zero. It is encoded according to DF402 from
# the RTCM 10403.2 Amendment 2 specification.  Valid values range
# from 0 to 15 and the most significant nibble is reserved for future use.
uint8 lock # []

# Measurement status flags. A bit field of flags providing the
# status of this observation.  If this field is 0 it means only the Cn0
# estimate for the signal is valid.
uint8 flags # []

# GNSS signal identifier (16 bit)
GnssSignal sid # []
