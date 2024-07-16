# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Settings
# MSG_SETTINGS_READ_REQ 0x00A4
#
# Read device configuration settings (host => device)
#
# The setting message that reads the device configuration. The string
# field is a NULL-terminated and NULL-delimited string with contents
# "SECTION_SETTING\0SETTING\0" where the '\0' escape sequence denotes the
# NULL character and where quotation marks are omitted. An example
# string that could be sent to a device is "solution\0soln_freq\0". A
# device will only respond to this message when it is received from
# sender ID 0x42. A device should respond with a MSG_SETTINGS_READ_RESP
# message (msg_id 0x00A5).

# A NULL-terminated and NULL-delimited string with contents
# "SECTION_SETTING\0SETTING\0"
string setting # []
