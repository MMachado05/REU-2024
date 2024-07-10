# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# MSG_SSR_ORBIT_CLOCK 0x05DD
#
# Precise orbit and clock correction
#
# The precise orbit and clock correction message is
# to be applied as a delta correction to broadcast
# ephemeris and is typically an equivalent to the 1060
# and 1066 RTCM message types

# GNSS reference time of the correction
GpsTimeSec time # []

# GNSS signal identifier (16 bit)
GnssSignal sid # []

# Update interval between consecutive corrections. Encoded
# following RTCM DF391 specification.
uint8 update_interval # []

# IOD of the SSR correction. A change of Issue Of Data
# SSR is used to indicate a change in the SSR
# generating configuration
uint8 iod_ssr # []

# Issue of broadcast ephemeris data or IODCRC (Beidou)
uint32 iod # []

# Orbit radial delta correction
int32 radial # [0.1 mm]

# Orbit along delta correction
int32 along # [0.4 mm]

# Orbit along delta correction
int32 cross # [0.4 mm]

# Velocity of orbit radial delta correction
int32 dot_radial # [0.001 mm/s]

# Velocity of orbit along delta correction
int32 dot_along # [0.004 mm/s]

# Velocity of orbit cross delta correction
int32 dot_cross # [0.004 mm/s]

# C0 polynomial coefficient for correction of broadcast satellite clock
int32 c0 # [0.1 mm]

# C1 polynomial coefficient for correction of broadcast satellite clock
int32 c1 # [0.001 mm/s]

# C2 polynomial coefficient for correction of broadcast satellite clock
int32 c2 # [0.00002 mm/s^-2]
