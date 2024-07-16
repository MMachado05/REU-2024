# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_ALMANAC_GLO 0x0073
#
# Satellite broadcast ephemeris for GLO
#
# The almanac message returns a set of satellite orbit parameters. Almanac
# data is not very precise and is considered valid for up to several months.
# Please see the GLO ICD 5.1 "Chapter 4.5 Non-immediate information and
# almanac" for details.

# Values common for all almanac types
AlmanacCommonContent common # []

# Longitude of the first ascending node of the orbit in PZ-90.02
# coordinate system
float64 lambda_na # [rad]

# Time of the first ascending node passage
float64 t_lambda_na # [s]

# Value of inclination at instant of t_lambda
float64 i # [rad]

# Value of Draconian period at instant of t_lambda
float64 t # [s/orbital period]

# Rate of change of the Draconian period
float64 t_dot # [s/(orbital period^2)]

# Eccentricity at instant of t_lambda
float64 epsilon # []

# Argument of perigee at instant of t_lambda
float64 omega # [rad]
