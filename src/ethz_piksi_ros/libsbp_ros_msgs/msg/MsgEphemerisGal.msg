# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_EPHEMERIS_GAL 0x008D
#
# Satellite broadcast ephemeris for Galileo
#
# The ephemeris message returns a set of satellite orbit
# parameters that is used to calculate Galileo satellite position,
# velocity, and clock offset. Please see the Signal In Space ICD
# OS SIS ICD, Issue 1.3, December 2016 for more details.

# Values common for all ephemeris types
EphemerisCommonContent common # []

# E1-E5a Broadcast Group Delay
float32 bgd_e1e5a # [s]

# E1-E5b Broadcast Group Delay
float32 bgd_e1e5b # [s]

# Amplitude of the sine harmonic correction term to the orbit radius
float32 c_rs # [m]

# Amplitude of the cosine harmonic correction term to the orbit radius
float32 c_rc # [m]

# Amplitude of the cosine harmonic correction term to the argument of latitude
float32 c_uc # [rad]

# Amplitude of the sine harmonic correction term to the argument of latitude
float32 c_us # [rad]

# Amplitude of the cosine harmonic correction term to the angle of inclination
float32 c_ic # [rad]

# Amplitude of the sine harmonic correction term to the angle of inclination
float32 c_is # [rad]

# Mean motion difference
float64 dn # [rad/s]

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

# Inclination first derivative
float64 inc_dot # [rad/s]

# Polynomial clock correction coefficient (clock bias)
float64 af0 # [s]

# Polynomial clock correction coefficient (clock drift)
float64 af1 # [s/s]

# Polynomial clock correction coefficient (rate of clock drift)
float32 af2 # [s/s^2]

# Clock reference
GpsTimeSec toc # []

# Issue of ephemeris data
uint16 iode # []

# Issue of clock data
uint16 iodc # []

# 0=I/NAV, 1=F/NAV, ...
uint8 source # []
