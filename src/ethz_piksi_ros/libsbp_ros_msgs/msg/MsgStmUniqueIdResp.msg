# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Flash
# MSG_STM_UNIQUE_ID_RESP 0x00E5
#
# Read device's hardcoded unique ID response (host <= device)

#
# This message reads the device's hardcoded unique ID. The host
# requests the ID by sending a MSG_STM_UNIQUE_ID_REQ. The device
# responds with a MSG_STM_UNIQUE_ID_RESP with the 12-byte unique
# ID in the payload..

# Device unique ID
uint8[] stm_id # []
