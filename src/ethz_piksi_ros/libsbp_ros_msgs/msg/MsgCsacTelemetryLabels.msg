# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# System
# MSG_CSAC_TELEMETRY_LABELS 0xFF05
#
# Experimental telemetry message labels
#
# The CSAC telemetry message provides labels for each member of the string
# produced by MSG_CSAC_TELEMETRY. It should be provided by a device at a lower
# rate than the MSG_CSAC_TELEMETRY.

# Index representing the type of telemetry in use.  It is implemention defined.
uint8 id # []

# Comma separated list of telemetry field values
string telemetry_labels # []
