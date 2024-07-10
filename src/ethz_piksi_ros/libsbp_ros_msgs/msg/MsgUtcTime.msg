# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_UTC_TIME 0x0103
#
# UTC Time
#
# This message reports the Universal Coordinated Time (UTC).  Note the flags
# which indicate the source of the UTC offset value and source of the time fix.

# Indicates source and time validity
uint8 flags # []

# GPS time of week rounded to the nearest millisecond
uint32 tow # [ms]

# Year
uint16 year # [year]

# Month (range 1 .. 12)
uint8 month # [months]

# days in the month (range 1-31)
uint8 day # [day]

# hours of day (range 0-23)
uint8 hours # [hours]

# minutes of hour (range 0-59)
uint8 minutes # [minutes]

# seconds of minute (range 0-60) rounded down
uint8 seconds # [seconds]

# nanoseconds of second (range 0-999999999)
uint32 ns # [nanoseconds]
