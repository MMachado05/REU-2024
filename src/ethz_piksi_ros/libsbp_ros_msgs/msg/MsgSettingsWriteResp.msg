# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Settings
# MSG_SETTINGS_WRITE_RESP 0x00AF
#
# Acknowledgement with status of MSG_SETTINGS_WRITE
#
# Return the status of a write request with the new value of the
# setting.  If the requested value is rejected, the current value
# will be returned. The string field is a NULL-terminated and NULL-delimited
# string with contents "SECTION_SETTING\0SETTING\0VALUE\0" where the '\0'
# escape sequence denotes the NULL character and where quotation marks
# are omitted. An example string that could be sent from device is
# "solution\0soln_freq\010\0".

# Write status
uint8 status # []

# A NULL-terminated and delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE\0" 
string setting # []
