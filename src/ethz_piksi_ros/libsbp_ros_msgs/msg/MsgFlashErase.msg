# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Flash
# MSG_FLASH_ERASE 0x00E2
#
# Erase sector of device flash memory (host => device).
#
# The flash erase message from the host erases a sector of either
# the STM or M25 onboard flash memory. The device will reply with a
# MSG_FLASH_DONE message containing the return code - FLASH_OK (0)
# on success or FLASH_INVALID_FLASH (1) if the flash specified is
# invalid.

# Target flags
uint8 target # []

# Flash sector number to erase (0-11 for the STM, 0-15 for
# the M25)
uint32 sector_num # []
