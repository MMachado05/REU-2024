# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Settings
# MSG_SETTINGS_READ_RESP 0x00A5
#
# Read device configuration settings (host <= device)
#
# The setting message wich which the device responds after a
# MSG_SETTING_READ_REQ is sent to device. The string field is a
# NULL-terminated and NULL-delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE\0" where the '\0' escape sequence
# denotes the NULL character and where quotation marks are omitted. An
# example string that could be sent from device is
# "solution\0soln_freq\010\0".

# A NULL-terminated and NULL-delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE\0"
#  
string setting # []
