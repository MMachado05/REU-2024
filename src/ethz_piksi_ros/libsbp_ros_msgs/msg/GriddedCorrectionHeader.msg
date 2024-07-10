# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# GriddedCorrectionHeader
#
# Header for MSG_SSR_GRIDDED_CORRECTION
#
# The 3GPP message contains nested variable length arrays
# which are not suppported in SBP, so each grid point will
# be identified by the index.

# GNSS reference time of the correction
GpsTimeSec time # []

# Number of messages in the dataset
uint16 num_msgs # []

# Position of this message in the dataset
uint16 seq_num # []

# Update interval between consecutive corrections. Encoded
# following RTCM DF391 specification.
uint8 update_interval # []

# IOD of the SSR atmospheric correction
uint8 iod_atmo # []

# Quality of the troposphere data. Encoded following RTCM DF389
# specifcation in units of m.
uint8 tropo_quality_indicator # []
