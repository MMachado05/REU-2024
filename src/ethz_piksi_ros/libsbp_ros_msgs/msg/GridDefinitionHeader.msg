# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# GridDefinitionHeader
#
# Defines the grid for MSG_SSR_GRIDDED_CORRECTION messages.
#
# Defines the grid for MSG_SSR_GRIDDED_CORRECTION messages.
# Also includes an RLE encoded validity list.

# region_size (deg) = 10 / region_size_inverse
# 0 is an invalid value.
uint8 region_size_inverse # [inverse degrees]

# grid height (deg) = grid idth (deg) = area_width / region_size
# 0 is an invalid value.
uint16 area_width # []

# North-West corner latitdue (deg) = region_size * lat_nw_corner_enc - 90
uint16 lat_nw_corner_enc # []

# North-West corner longtitude (deg) = region_size * lon_nw_corner_enc - 180
uint16 lon_nw_corner_enc # []

# Number of messages in the dataset
uint8 num_msgs # []

# Postion of this message in the dataset
uint8 seq_num # []
