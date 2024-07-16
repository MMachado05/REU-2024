# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Bootload
# MSG_NAP_DEVICE_DNA_RESP 0x00DD
#
# Read FPGA device ID over UART response (host <= device)
#
# The device message from the host reads a unique device
# identifier from the SwiftNAP, an FPGA. The host requests the ID
# by sending a MSG_NAP_DEVICE_DNA_REQ message. The device
# responds with a MSG_NAP_DEVICE_DNA_RESP messagage with the
# device ID in the payload. Note that this ID is tied to the FPGA,
# and not related to the Piksi's serial number.

# 57-bit SwiftNAP FPGA Device ID. Remaining bits are padded
# on the right.
uint8[] dna # []
