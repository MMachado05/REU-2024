# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# MSG_FRONT_END_GAIN 0x00BF
#
# RF AGC status
#
# This message describes the gain of each channel in the receiver frontend. Each 
# gain is encoded as a non-dimensional percentage relative to the maximum range  
# possible for the gain stage of the frontend. By convention, each gain array 
# has 8 entries and the index of the array corresponding to the index of the rf channel 
# in the frontend. A gain of 127 percent encodes that rf channel is not present in the hardware.
# A negative value implies an error for the particular gain stage as reported by the frontend.

# RF gain for each frontend channel
int8[] rf_gain # [percent]

# Intermediate frequency gain for each frontend channel
int8[] if_gain # [percent]
