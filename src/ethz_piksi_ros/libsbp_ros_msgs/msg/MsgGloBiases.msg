# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Observation
# MSG_GLO_BIASES 0x0075
#
# GLONASS L1/L2 Code-Phase biases
#
# The GLONASS L1/L2 Code-Phase biases allows to perform
# GPS+GLONASS integer ambiguity resolution for baselines
# with mixed receiver types (e.g. receiver of different
# manufacturers)

# GLONASS FDMA signals mask
uint8 mask # [boolean]

# GLONASS L1 C/A Code-Phase Bias
int16 l1ca_bias # [m * 0.02]

# GLONASS L1 P Code-Phase Bias
int16 l1p_bias # [m * 0.02]

# GLONASS L2 C/A Code-Phase Bias
int16 l2ca_bias # [m * 0.02]

# GLONASS L2 P Code-Phase Bias
int16 l2p_bias # [m * 0.02]
