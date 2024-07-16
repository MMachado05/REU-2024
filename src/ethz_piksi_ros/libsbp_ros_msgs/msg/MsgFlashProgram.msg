# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Flash
# MSG_FLASH_PROGRAM 0x00E6
#
# Program flash addresses
#
# The flash program message programs a set of addresses of either
# the STM or M25 flash. The device replies with either a
# MSG_FLASH_DONE message containing the return code FLASH_OK (0)
# on success, or FLASH_INVALID_LEN (2) if the maximum write size
# is exceeded. Note that the sector-containing addresses must be
# erased before addresses can be programmed.

# Target flags
uint8 target # []

# Starting address offset to program
uint8[] addr_start # [bytes]

# Length of set of addresses to program, counting up from
# starting address
uint8 addr_len # [bytes]

# Data to program addresses with, with length N=addr_len
uint8[] data # []
