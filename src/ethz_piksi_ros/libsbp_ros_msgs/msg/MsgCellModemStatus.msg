# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# MSG_CELL_MODEM_STATUS 0x00BE
#
# Cell modem information update message
#
# If a cell modem is present on a piksi device, this message
# will be send periodically to update the host on the status
# of the modem and its various parameters.

# Received cell signal strength in dBm, zero translates to unknown
int8 signal_strength # [dBm]

# BER as reported by the modem, zero translates to unknown
float32 signal_error_rate # []

