# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# UARTChannel
#
# State of the UART channel
#
# Throughput, utilization, and error counts on the RX/TX buffers
# of this UART channel. The reported percentage values must
# be normalized.

# UART transmit throughput
float32 tx_throughput # [kB/s]

# UART receive throughput
float32 rx_throughput # [kB/s]

# UART CRC error count
uint16 crc_error_count # []

# UART IO error count
uint16 io_error_count # []

# UART transmit buffer percentage utilization (ranges from
# 0 to 255)
uint8 tx_buffer_level # []

# UART receive buffer percentage utilization (ranges from
# 0 to 255)
uint8 rx_buffer_level # []
