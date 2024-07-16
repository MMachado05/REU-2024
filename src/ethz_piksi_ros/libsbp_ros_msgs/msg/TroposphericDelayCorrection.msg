# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# TroposphericDelayCorrection
#
# None
#
# Troposphere vertical delays (mean and standard deviation) at the grid
# point.

# Hydrostatic vertical delay
int16 hydro # [4 mm (add 2.3 m to get actual vertical hydro delay)]

# Wet vertical delay
int8 wet # [4 mm (add 0.252 m to get actual vertical wet delay)]

# stddev
uint8 stddev # [modified DF389 scale; class is upper 3 bits, value is lower 5
# stddev <= (3^class * (1 + value/16) - 1) mm
