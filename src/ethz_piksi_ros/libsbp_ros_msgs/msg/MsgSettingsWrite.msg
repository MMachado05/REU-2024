# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Settings
# MSG_SETTINGS_WRITE 0x00A0
#
# Write device configuration settings (host => device)
#
# The setting message writes the device configuration for a particular
# setting via A NULL-terminated and NULL-delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE\0" where the '\0' escape sequence denotes 
# the NULL character and where quotation marks are omitted. A device will
# only process to this message when it is received from sender ID 0x42.
# An example string that could be sent to a device is
# "solution\0soln_freq\010\0".

# A NULL-terminated and NULL-delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE\0"
string setting # []
