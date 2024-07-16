# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# MSG_NETWORK_STATE_RESP 0x00BB
#
# State of network interface
#
# The state of a network interface on the Piksi.
# Data is made to reflect output of ifaddrs struct returned by getifaddrs
# in c.

# IPv4 address (all zero when unavailable)
uint8[] ipv4_address # []

# IPv4 netmask CIDR notation
uint8 ipv4_mask_size # []

# IPv6 address (all zero when unavailable)
uint8[] ipv6_address # []

# IPv6 netmask CIDR notation
uint8 ipv6_mask_size # []

# Number of Rx bytes
uint32 rx_bytes # []

# Number of Tx bytes
uint32 tx_bytes # []

# Interface Name
string interface_name # []

# Interface flags from SIOCGIFFLAGS
uint32 flags # []
