# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_ALMANAC_GPS 0x0072
#
# Satellite broadcast ephemeris for GPS
#
# The almanac message returns a set of satellite orbit parameters. Almanac
# data is not very precise and is considered valid for up to several months.
# Please see the Navstar GPS Space Segment/Navigation user interfaces
# (ICD-GPS-200, Chapter 20.3.3.5.1.2 Almanac Data) for more details.

# Values common for all almanac types
AlmanacCommonContent common # []

# Mean anomaly at reference time
float64 m0 # [rad]

# Eccentricity of satellite orbit
float64 ecc # []

# Square root of the semi-major axis of orbit
float64 sqrta # [m^(1/2)]

# Longitude of ascending node of orbit plane at weekly epoch
float64 omega0 # [rad]

# Rate of right ascension
float64 omegadot # [rad/s]

# Argument of perigee
float64 w # [rad]

# Inclination
float64 inc # [rad]

# Polynomial clock correction coefficient (clock bias)
float64 af0 # [s]

# Polynomial clock correction coefficient (clock drift)
float64 af1 # [s/s]
