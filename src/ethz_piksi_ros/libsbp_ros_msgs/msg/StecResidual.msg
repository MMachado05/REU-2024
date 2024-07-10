# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# STECResidual
#
# None
#
# STEC residual (mean and standard deviation) for the given satellite
# at the grid point,

# space vehicle identifier
SvId sv_id # []

# STEC residual
int16 residual # [0.04 TECU]

# stddev
uint8 stddev # [modified DF389 scale; class is upper 3 bits, value is lower 5
# stddev <= (3^class * (1 + value/16) - 1) * 10 TECU
