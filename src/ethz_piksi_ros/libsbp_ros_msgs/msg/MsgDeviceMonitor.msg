# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Piksi
# MSG_DEVICE_MONITOR 0x00B5
#
# Device temperature and voltage levels
#
# This message contains temperature and voltage level measurements from the
# processor's monitoring system and the RF frontend die temperature if
# available.

# Device V_in
int16 dev_vin # [V / 1000]

# Processor V_int
int16 cpu_vint # [V / 1000]

# Processor V_aux
int16 cpu_vaux # [V / 1000]

# Processor temperature
int16 cpu_temperature # [degrees C / 100]

# Frontend temperature (if available)
int16 fe_temperature # [degrees C / 100]
