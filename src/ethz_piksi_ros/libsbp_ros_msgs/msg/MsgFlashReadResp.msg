# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Flash
# MSG_FLASH_READ_RESP 0x00E1
#
# Read STM or M25 flash address response (host <= device).
#
# The flash read message reads a set of addresses of either the
# STM or M25 onboard flash. The device replies with a
# MSG_FLASH_READ_RESP message containing either the read data on
# success or a MSG_FLASH_DONE message containing the return code
# FLASH_INVALID_LEN (2) if the maximum read size is exceeded or
# FLASH_INVALID_ADDR (3) if the address is outside of the allowed
# range.

# Target flags
uint8 target # []

# Starting address offset to read from
uint8[] addr_start # [bytes]

# Length of set of addresses to read, counting up from
# starting address
uint8 addr_len # [bytes]
