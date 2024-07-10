# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# Period
#
# base station observation message receipt period
#
# Statistics on the period of observations received from the base
# station. As complete observation sets are received, their time
# of reception is compared with the prior set''s time of reception.
# This measurement provides a proxy for link quality as incomplete
# or missing sets will increase the period.  Long periods
# can cause momentary RTK solution outages.

# Average period
int32 avg # [ms]

# Minimum period
int32 pmin # [ms]

# Maximum period
int32 pmax # [ms]

# Smoothed estimate of the current period
int32 current # [ms]
