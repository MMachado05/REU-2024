# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# Doppler
#
# GNSS doppler measurement.
#
# Doppler measurement in Hz represented as a 24-bit
# fixed point number with Q16.8 layout, i.e. 16-bits of whole
# doppler and 8-bits of fractional doppler. This doppler is defined
# as positive for approaching satellites.

# Doppler whole Hz
int16 i # [Hz]

# Doppler fractional part
uint8 f # [Hz / 256]
