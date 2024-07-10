# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# STECHeader
#
# Header for MSG_SSR_STEC_CORRECTION message
#
# A full set of STEC information will likely span multiple SBP
# messages, since SBP message a limited to 255 bytes.  The header
# is used to tie multiple SBP messages into a sequence.

# GNSS reference time of the correction
GpsTimeSec time # []

# Number of messages in the dataset
uint8 num_msgs # []

# Position of this message in the dataset
uint8 seq_num # []

# Update interval between consecutive corrections. Encoded
# following RTCM DF391 specification.
uint8 update_interval # []

# IOD of the SSR atmospheric correction
uint8 iod_atmo # []
