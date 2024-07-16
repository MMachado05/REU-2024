# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Sbas
# MSG_SBAS_RAW 0x7777
#
# Raw SBAS data
#
# This message is sent once per second per SBAS satellite. ME checks the
# parity of the data block and sends only blocks that pass the check.

# GNSS signal identifier.
GnssSignal sid # []

# GPS time-of-week at the start of the data block.
uint32 tow # [ms]

# SBAS message type (0-63)
uint8 message_type # []

# Raw SBAS data field of 212 bits (last byte padded with zeros).
uint8[] data # []
