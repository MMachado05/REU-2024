# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Flash
# MSG_STM_FLASH_UNLOCK_SECTOR 0x00E4
#
# Unlock sector of STM flash memory (host => device)
#
# The flash unlock message unlocks a sector of the STM flash
# memory. The device replies with a MSG_FLASH_DONE message.

# Flash sector number to unlock
uint32 sector # []
