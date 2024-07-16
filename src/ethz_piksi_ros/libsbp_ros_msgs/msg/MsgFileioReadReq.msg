# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# File_Io
# MSG_FILEIO_READ_REQ 0x00A8
#
# Read file from the file system (host => device)
#
# The file read message reads a certain length (up to 255 bytes)
# from a given offset into a file, and returns the data in a
# MSG_FILEIO_READ_RESP message where the message length field
# indicates how many bytes were succesfully read.The sequence
# number in the request will be returned in the response.
# If the message is invalid, a followup MSG_PRINT message will
# print "Invalid fileio read message". A device will only respond
# to this message when it is received from sender ID 0x42.

# Read sequence number
uint32 sequence # []

# File offset
uint32 offset # [bytes]

# Chunk size to read
uint8 chunk_size # [bytes]

# Name of the file to read from
string filename # []
