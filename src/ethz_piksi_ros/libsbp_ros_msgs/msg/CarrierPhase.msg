# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Gnss
# CarrierPhase
#
# GNSS carrier phase measurement.
#
# Carrier phase measurement in cycles represented as a 40-bit
# fixed point number with Q32.8 layout, i.e. 32-bits of whole
# cycles and 8-bits of fractional cycles. This phase has the
# same sign as the pseudorange.

# Carrier phase whole cycles
int32 i # [cycles]

# Carrier phase fractional part
uint8 f # [cycles / 256]
