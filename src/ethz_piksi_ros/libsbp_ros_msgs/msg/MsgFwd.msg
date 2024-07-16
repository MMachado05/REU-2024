# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Logging
# MSG_FWD 0x0402
#
# Wrapper for FWD a separate stream of information over SBP
#
# This message provides the ability to forward messages over SBP.  This may take the form
# of wrapping up SBP messages received by Piksi for logging purposes or wrapping 
# another protocol with SBP.
# 
# The source identifier indicates from what interface a forwarded stream derived.
# The protocol identifier identifies what the expected protocol the forwarded msg contains.
# Protocol 0 represents SBP and the remaining values are implementation defined.

# source identifier
uint8 source # []

# protocol identifier
uint8 protocol # []

# variable length wrapped binary message
string fwd_payload # []
