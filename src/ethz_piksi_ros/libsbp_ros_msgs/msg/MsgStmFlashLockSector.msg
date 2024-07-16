# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Flash
# MSG_STM_FLASH_LOCK_SECTOR 0x00E3
#
# Lock sector of STM flash memory (host => device)
#
# The flash lock message locks a sector of the STM flash
# memory. The device replies with a MSG_FLASH_DONE message.

# Flash sector number to lock
uint32 sector # []
