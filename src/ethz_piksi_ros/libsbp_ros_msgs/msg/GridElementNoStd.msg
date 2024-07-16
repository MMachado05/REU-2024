# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# GridElementNoStd
#
# Correction data for a single grid point.
#
# Contains one tropo delay, plus STEC residuals for each satellite at the
# grid point.

# Index of the grid point
uint16 index # []

# Wet and hydrostatic vertical delays
TroposphericDelayCorrectionNoStd tropo_delay_correction # []

# STEC residuals for each satellite
StecResidualNoStd[] stec_residuals # []
