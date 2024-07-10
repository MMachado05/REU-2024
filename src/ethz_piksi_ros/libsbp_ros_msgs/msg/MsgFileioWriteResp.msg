# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# File_Io
# MSG_FILEIO_WRITE_RESP 0x00AB
#
# File written to (host <= device)
#
# The file write message writes a certain length (up to 255 bytes)
# of data to a file at a given offset. The message is a copy of the
# original MSG_FILEIO_WRITE_REQ message to check integrity of the
# write. The sequence number in the response is preserved from the
# request.

# Write sequence number
uint32 sequence # []
