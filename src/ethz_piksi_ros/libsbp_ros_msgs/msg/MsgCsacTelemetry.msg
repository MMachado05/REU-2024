# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# System
# MSG_CSAC_TELEMETRY 0xFF04
#
# Experimental telemetry message
#
# The CSAC telemetry message has an implementation defined telemetry string
# from a device. It is not produced or available on general Swift Products.
# It is intended to be a low rate message for status purposes.

# Index representing the type of telemetry in use.  It is implemention defined.
uint8 id # []

# Comma separated list of values as defined by the index
string telemetry # []
