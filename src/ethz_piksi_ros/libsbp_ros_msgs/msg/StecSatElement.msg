# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# STECSatElement
#
# None
#
# STEC polynomial for the given satellite.

# Unique space vehicle identifier
SvId sv_id # []

# Quality of the STEC data. Encoded following RTCM DF389 specifcation
# but in units of TECU instead of m.
uint8 stec_quality_indicator # []

# Coefficents of the STEC polynomial in the order of C00, C01, C10, C11
int16[] stec_coeff # [C00 = 0.05 TECU, others = 0.02 TECU/deg]
