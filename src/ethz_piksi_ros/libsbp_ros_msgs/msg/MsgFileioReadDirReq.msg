# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# File_Io
# MSG_FILEIO_READ_DIR_REQ 0x00A9
#
# List files in a directory (host => device)
#
# The read directory message lists the files in a directory on the
# device's onboard flash file system.  The offset parameter can be
# used to skip the first n elements of the file list. Returns a
# MSG_FILEIO_READ_DIR_RESP message containing the directory
# listings as a NULL delimited list. The listing is chunked over
# multiple SBP packets. The sequence number in the request will be
# returned in the response.  If message is invalid, a followup
# MSG_PRINT message will print "Invalid fileio read message".
# A device will only respond to this message when it is received
# from sender ID 0x42.

# Read sequence number
uint32 sequence # []

# The offset to skip the first n elements of the file list
uint32 offset # []

# Name of the directory to list
string dirname # []
