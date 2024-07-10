# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# MSG_COMMAND_REQ 0x00B8
#
# Execute a command (host => device)
#
# Request the recipient to execute an command.
# Output will be sent in MSG_LOG messages, and the exit
# code will be returned with MSG_COMMAND_RESP.

# Sequence number
uint32 sequence # []

# Command line to execute
string command # []
