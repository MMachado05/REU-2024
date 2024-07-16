# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Navigation
# MSG_DOPS 0x0208
#
# Dilution of Precision
#
# This dilution of precision (DOP) message describes the effect of
# navigation satellite geometry on positional measurement
# precision.  The flags field indicated whether the DOP reported
# corresponds to differential or SPP solution.

# GPS Time of Week
uint32 tow # [ms]

# Geometric Dilution of Precision
uint16 gdop # [0.01]

# Position Dilution of Precision
uint16 pdop # [0.01]

# Time Dilution of Precision
uint16 tdop # [0.01]

# Horizontal Dilution of Precision
uint16 hdop # [0.01]

# Vertical Dilution of Precision
uint16 vdop # [0.01]

# Indicates the position solution with which the DOPS message corresponds
uint8 flags # []
