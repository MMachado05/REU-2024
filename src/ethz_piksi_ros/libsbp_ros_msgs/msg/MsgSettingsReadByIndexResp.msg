# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Settings
# MSG_SETTINGS_READ_BY_INDEX_RESP 0x00A7
#
# Read setting by direct index (host <= device)
#
# The settings message that reports the value of a setting at an index.
# 
# In the string field, it reports NULL-terminated and delimited string
# with contents "SECTION_SETTING\0SETTING\0VALUE\0FORMAT_TYPE\0". where
# the '\0' escape sequence denotes the NULL character and where quotation
# marks are omitted. The FORMAT_TYPE field is optional and denotes
# possible string values of the setting as a hint to the user. If
# included, the format type portion of the string has the format
# "enum:value1,value2,value3". An example string that could be sent from
# the device is "simulator\0enabled\0True\0enum:True,False\0"

# An index into the device settings, with values ranging from
# 0 to length(settings)
uint16 index # []

# A NULL-terminated and delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE\0FORMAT_TYPE\0"
string setting # []
