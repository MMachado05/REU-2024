# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Gnss
# GPSTime
#
# Nanosecond-accurate receiver clock time
#
# A wire-appropriate receiver clock time, defined as the time
# since the beginning of the week on the Saturday/Sunday
# transition. In most cases, observations are epoch aligned
# so ns field will be 0.

# Milliseconds since start of GPS week
uint32 tow # [ms]

# Nanosecond residual of millisecond-rounded TOW (ranges
# from -500000 to 500000)
int32 ns_residual # [ns]

# GPS week number
uint16 wn # [week]
