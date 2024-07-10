# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# MSG_SSR_GRID_DEFINITION 0x05F5
#
# Definition of the grid for STEC and tropo messages
#
# Based on the 3GPP proposal R2-1906781 which is in turn based on
# OMA-LPPe-ValidityArea from OMA-TS-LPPe-V2_0-20141202-C

# Header of a Gridded Correction message
GridDefinitionHeader header # []

# Run Length Encode list of quadrants that contain valid data.
# The spec describes the encoding scheme in detail, but
# essentially the index of the quadrants that contain transitions between
# valid and invalid (and vice versa) are encoded as u8 integers.
uint8[] rle_list # []
