# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Settings
# MSG_SETTINGS_READ_BY_INDEX_REQ 0x00A2
#
# Read setting by direct index (host => device)
#
# The settings message for iterating through the settings
# values. A device will respond to this message with a 
# "MSG_SETTINGS_READ_BY_INDEX_RESP".

# An index into the device settings, with values ranging from
# 0 to length(settings)
uint16 index # []
