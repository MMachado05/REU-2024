# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Settings
# MSG_SETTINGS_REGISTER 0x00AE
#
# Register setting and default value (device => host)
#
# This message registers the presence and default value of a setting
# with a settings daemon.  The host should reply with MSG_SETTINGS_WRITE
# for this setting to set the initial value.

# A NULL-terminated and delimited string with contents
# "SECTION_SETTING\0SETTING\0VALUE".
string setting # []
