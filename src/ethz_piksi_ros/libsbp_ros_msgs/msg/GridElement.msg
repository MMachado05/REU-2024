# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Ssr
# GridElement
#
# Correction data for a single grid point.
#
# Contains one tropo delay (mean and stddev), plus STEC residuals (mean and
# stddev) for each satellite at the grid point.

# Index of the grid point
uint16 index # []

# Wet and hydrostatic vertical delays (mean, stddev)
TroposphericDelayCorrection tropo_delay_correction # []

# STEC residuals for each satellite (mean, stddev)
StecResidual[] stec_residuals # []
