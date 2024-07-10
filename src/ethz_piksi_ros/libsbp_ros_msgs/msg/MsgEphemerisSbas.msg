# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_EPHEMERIS_SBAS 0x008C
#
# Satellite broadcast ephemeris for SBAS
#


# Values common for all ephemeris types
EphemerisCommonContent common # []

# Position of the GEO at time toe
float64[] pos # [m]

# Velocity of the GEO at time toe
float32[] vel # [m/s]

# Acceleration of the GEO at time toe
float32[] acc # [m/s^2]

# Time offset of the GEO clock w.r.t. SBAS Network Time
float32 a_gf0 # [s]

# Drift of the GEO clock w.r.t. SBAS Network Time
float32 a_gf1 # [s/s]
