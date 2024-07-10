# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Acquisition
# MSG_ACQ_RESULT 0x002F
#
# Satellite acquisition result
#
# This message describes the results from an attempted GPS signal
# acquisition search for a satellite PRN over a code phase/carrier
# frequency range. It contains the parameters of the point in the
# acquisition search space with the best carrier-to-noise (CN/0)
# ratio.

# CN/0 of best point
float32 cn0 # [dB Hz]

# Code phase of best point
float32 cp # [chips]

# Carrier frequency of best point
float32 cf # [hz]

# GNSS signal for which acquisition was attempted
GnssSignal sid # []
