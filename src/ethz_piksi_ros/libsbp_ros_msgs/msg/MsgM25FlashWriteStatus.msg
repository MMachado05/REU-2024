# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Flash
# MSG_M25_FLASH_WRITE_STATUS 0x00F3
#
# Write M25 flash status register (host => device)
#
# The flash status message writes to the 8-bit M25 flash status
# register. The device replies with a MSG_FLASH_DONE message.

# Byte to write to the M25 flash status register
uint8[] status # []
