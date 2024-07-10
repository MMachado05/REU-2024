# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# File_Io
# MSG_FILEIO_CONFIG_RESP 0x1002
#
# Response with advice on the optimal configuration for FileIO.

#
# The advice on the optimal configuration for a FileIO
# transfer.  Newer version of FileIO can support greater
# throughput by supporting a large window of FileIO data
# that can be in-flight during read or write operations.

# Advice sequence number
uint32 sequence # []

# The number of SBP packets in the data in-flight window
uint32 window_size # []

# The number of SBP packets sent in one PDU
uint32 batch_size # []

# The version of FileIO that is supported
uint32 fileio_version # []
