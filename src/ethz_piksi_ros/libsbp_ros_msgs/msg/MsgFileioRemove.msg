# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# File_Io
# MSG_FILEIO_REMOVE 0x00AC
#
# Delete a file from the file system (host => device)
#
# The file remove message deletes a file from the file system.
# If the message is invalid, a followup MSG_PRINT message will
# print "Invalid fileio remove message". A device will only
# process this message when it is received from sender ID 0x42.

# Name of the file to delete
string filename # []
