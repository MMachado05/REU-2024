# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# MSG_SPECAN 0x0051
#
# Spectrum analyzer
#
# Spectrum analyzer packet.

# Channel ID
uint16 channel_tag # []

# Receiver time of this observation
GpsTime t # []

# Reference frequency of this packet
float32 freq_ref # [MHz]

# Frequency step of points in this packet
float32 freq_step # [MHz]

# Reference amplitude of this packet
float32 amplitude_ref # [dB]

# Amplitude unit value of points in this packet
float32 amplitude_unit # [dB]

# Amplitude values (in the above units) of points in this packet
uint8[] amplitude_value # []
