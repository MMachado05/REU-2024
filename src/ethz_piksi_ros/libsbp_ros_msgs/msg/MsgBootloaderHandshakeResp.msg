# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Bootload
# MSG_BOOTLOADER_HANDSHAKE_RESP 0x00B4
#
# Bootloading handshake response (host <= device)
#
# The handshake message response from the device establishes a
# handshake between the device bootloader and the host. The
# request from the host is MSG_BOOTLOADER_HANDSHAKE_REQ.  The
# payload contains the bootloader version number and the SBP
# protocol version number.

# Bootloader flags
uint32 flags # []

# Bootloader version number
string version # []
