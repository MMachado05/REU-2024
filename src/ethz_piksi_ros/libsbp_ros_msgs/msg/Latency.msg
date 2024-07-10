# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# Latency
#
# Receiver-to-base station latency
#
# Statistics on the latency of observations received from the base
# station. As observation packets are received their GPS time is
# compared to the current GPS time calculated locally by the
# receiver to give a precise measurement of the end-to-end
# communication latency in the system.

# Average latency
int32 avg # [ms]

# Minimum latency
int32 lmin # [ms]

# Maximum latency
int32 lmax # [ms]

# Smoothed estimate of the current latency
int32 current # [ms]
