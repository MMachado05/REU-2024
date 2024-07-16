# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Flash
# MSG_FLASH_DONE 0x00E0
#
# Flash response message (host <= device).
#
# This message defines success or failure codes for a variety of
# flash memory requests from the host to the device. Flash read
# and write messages, such as MSG_FLASH_READ_REQ, or
# MSG_FLASH_PROGRAM, may return this message on failure.

# Response flags
uint8 response # []
