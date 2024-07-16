# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ndb
# MSG_NDB_EVENT 0x0400
#
# Navigation DataBase Event
#
# This message is sent out when an object is stored into NDB. If needed
# message could also be sent out when fetching an object from NDB.

# HW time in milliseconds.
uint64 recv_time # [ms]

# Event type.
uint8 event # []

# Event object type.
uint8 object_type # []

# Event result.
uint8 result # []

# Data source for STORE event, reserved for other events.
uint8 data_source # []

# GNSS signal identifier,
# If object_type is Ephemeris OR Almanac, sid indicates for which
# signal the object belongs to. Reserved in other cases.
GnssSignal object_sid # []

# GNSS signal identifier,
# If object_type is Almanac, Almanac WN, Iono OR L2C capabilities
# AND data_source is NDB_DS_RECEIVER sid indicates from which SV
# data was decoded. Reserved in other cases.
GnssSignal src_sid # []

# A unique identifier of the sending hardware. For v1.0,
# set to the 2 least significant bytes of the device serial
# number, valid only if data_source is NDB_DS_SBP. Reserved in case
# of other data_source.
uint16 original_sender # []
