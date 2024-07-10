# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# File_Io
# MSG_FILEIO_READ_RESP 0x00A3
#
# File read from the file system (host <= device)
#
# The file read message reads a certain length (up to 255 bytes)
# from a given offset into a file, and returns the data in a
# message where the message length field indicates how many bytes
# were succesfully read. The sequence number in the response is
# preserved from the request.

# Read sequence number
uint32 sequence # []

# Contents of read file
uint8[] contents # []
