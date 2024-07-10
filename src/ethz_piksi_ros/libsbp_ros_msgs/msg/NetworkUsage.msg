# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# NetworkUsage
#
# Bandwidth usage measurement for a single interface.
#
# The bandwidth usage for each interface can be reported
# within this struct and utilize multiple fields to fully
# specify the type of traffic that is being tracked. As
# either the interval of collection or the collection time
# may vary, both a timestamp and period field is provided,
# though may not necessarily be populated with a value. 

# Duration over which the measurement was collected
uint64 duration # [ms]

# Number of bytes handled in total within period
uint64 total_bytes # []

# Number of bytes transmitted within period
uint32 rx_bytes # []

# Number of bytes received within period
uint32 tx_bytes # []

# Interface Name
string interface_name # []
