# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# MSG_UART_STATE 0x001D
#
# State of the UART channels
#
# The UART message reports data latency and throughput of the UART
# channels providing SBP I/O. On the default Piksi configuration,
# UARTs A and B are used for telemetry radios, but can also be
# host access ports for embedded hosts, or other interfaces in
# future. The reported percentage values must be normalized.
# Observations latency and period can be used to assess the
# health of the differential corrections link. Latency provides
# the timeliness of received base observations while the
# period indicates their likelihood of transmission.

# State of UART A
UartChannel uart_a # []

# State of UART B
UartChannel uart_b # []

# State of UART FTDI (USB logger)
UartChannel uart_ftdi # []

# UART communication latency
Latency latency # []

# Observation receipt period
Period obs_period # []
