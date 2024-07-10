# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Acquisition
# AcqSvProfile
#
# Acq perfomance measurement and debug
#
# Profile for a specific SV for debugging purposes
# The message describes SV profile during acquisition time.
# The message is used to debug and measure the performance.

# SV search job type (deep, fallback, etc)
uint8 job_type # []

# Acquisition status 1 is Success, 0 is Failure
uint8 status # []

# CN0 value. Only valid if status is '1'
uint16 cn0 # [dB-Hz*10]

# Acquisition integration time
uint8 int_time # [ms]

# GNSS signal for which acquisition was attempted
GnssSignal sid # []

# Acq frequency bin width
uint16 bin_width # [Hz]

# Timestamp of the job complete event
uint32 timestamp # [ms]

# Time spent to search for sid.code
uint32 time_spent # [us]

# Doppler range lowest frequency
int32 cf_min # [Hz]

# Doppler range highest frequency
int32 cf_max # [Hz]

# Doppler value of detected peak. Only valid if status is '1'
int32 cf # [Hz]

# Codephase of detected peak. Only valid if status is '1'
uint32 cp # [chips*10]
