### Group for XL30 small current 30keV layers ###
#################################################

import numpy as np

from ..design_layer import DesignLayer, DesignLayerType
from ..design_layer_taxonomy import DESIGN_LAYER_TYPE_TAXONOMY
from ..design_layer_group import DesignLayerGroup
from ..design_layer_group_policy import EbeamDesignGroup
from ..design_layer_set import DesignLayerSet


ebeam_layers = []

## add layers to ebeam layer group

### Exposure layers ###
#######################

EBEAM_DOSE = 2300
EBEAM_LAYER_NUMBER = 5
ebeam_layer = DesignLayer(
                    name = 'ebeam',
                    gds_layer = EBEAM_LAYER_NUMBER,
                    gds_datatype = EBEAM_DOSE,
                    description = 'layer for ebeam lithography',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(ebeam_layer)

EBEAM_VERY_LOW_DOSE = 1800
EBEAM_VERY_LOW_LAYER_NUMBER = 6
ebeam_very_low_layer = DesignLayer(
                    name = 'ebeam_very_low',
                    gds_layer = EBEAM_VERY_LOW_LAYER_NUMBER,
                    gds_datatype = EBEAM_VERY_LOW_DOSE,
                    description = 'layer for very low dose ebeam lithography',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(ebeam_very_low_layer)

EBEAM_LOW_DOSE = 2100
EBEAM_LOW_LAYER_NUMBER = 7
ebeam_low_layer = DesignLayer(
                    name = 'ebeam_low',
                    gds_layer = EBEAM_LOW_LAYER_NUMBER,
                    gds_datatype = EBEAM_LOW_DOSE,
                    description = 'layer for low dose ebeam lithography',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(ebeam_low_layer)

EBEAM_HIGH_DOSE = 2800
EBEAM_HIGH_LAYER_NUMBER = 8
ebeam_high_layer = DesignLayer(
                    name = 'ebeam_high',
                    gds_layer = EBEAM_HIGH_LAYER_NUMBER,
                    gds_datatype = EBEAM_HIGH_DOSE,
                    description = 'layer for high dose ebeam lithography',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(ebeam_high_layer)

EBEAM_SMALL_JUNCTIONS_DOSE = 2800
EBEAM_SMALL_JUNCTIONS_LAYER_NUMBER = 10
ebeam_small_junctions_layer = DesignLayer(
                    name = 'ebeam_small_junctions',
                    gds_layer = EBEAM_SMALL_JUNCTIONS_LAYER_NUMBER,
                    gds_datatype = EBEAM_SMALL_JUNCTIONS_DOSE,
                    description = 'layer for small junction ebeam lithography',
                    color = 'darkblue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(ebeam_small_junctions_layer)

EBEAM_JUNCTIONS_DOSE = 2500
EBEAM_JUNCTIONS_LAYER_NUMBER = 11
ebeam_junctions_layer = DesignLayer(
                    name = 'ebeam_junctions',
                    gds_layer = EBEAM_JUNCTIONS_LAYER_NUMBER,
                    gds_datatype = EBEAM_JUNCTIONS_DOSE,
                    description = 'layer for normal junction ebeam lithography',
                    color = 'darkblue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(ebeam_junctions_layer)

EBEAM_LARGE_JUNCTIONS_DOSE = 1800
EBEAM_LARGE_JUNCTIONS_LAYER_NUMBER = 12
ebeam_large_junctions_layer = DesignLayer(
                    name = 'ebeam_large_junctions',
                    gds_layer = EBEAM_LARGE_JUNCTIONS_LAYER_NUMBER,
                    gds_datatype = EBEAM_LARGE_JUNCTIONS_DOSE,
                    description = 'layer for large junction ebeam lithography',
                    color = 'darkblue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(ebeam_large_junctions_layer)

EBEAM_JUNCTIONS_STRESS_DOSE = 2000
EBEAM_JUNCTIONS_STRESS_LAYER_NUMBER = 13
ebeam_junctions_stress_layer = DesignLayer(
                    name = 'ebeam_junctions_stress',
                    gds_layer = EBEAM_JUNCTIONS_STRESS_LAYER_NUMBER,
                    gds_datatype = EBEAM_JUNCTIONS_STRESS_DOSE,
                    description = 'layer for junction stress relief structures ebeam lithography',
                    color = 'darkblue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(ebeam_junctions_stress_layer)

JJ_UNDERCUT_DOSE = 460
JJ_UNDERCUT_LAYER_NUMBER = 14
jj_undercut_layer = DesignLayer(
                    name = 'jj_undercut',
                    gds_layer = JJ_UNDERCUT_LAYER_NUMBER,
                    gds_datatype = JJ_UNDERCUT_DOSE,
                    description = 'layer for undercut',
                    color = 'red',
                    alpha = 0.8,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(jj_undercut_layer)

UNDERCUT_DOSE = 450
UNDERCUT_LAYER_NUMBER = 15
undercut_layer = DesignLayer(
                    name = 'undercut',
                    gds_layer = UNDERCUT_LAYER_NUMBER,
                    gds_datatype = UNDERCUT_DOSE,
                    description = 'layer for ebeam undercut',
                    color = 'darkblue',
                    alpha = 0.8,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(undercut_layer)

UNDERCUT_VERY_LOW_DOSE = 390
UNDERCUT_VERY_LOW_LAYER_NUMBER = 16
undercut_very_low_layer = DesignLayer(
                    name = 'undercut_very_low',
                    gds_layer = UNDERCUT_VERY_LOW_LAYER_NUMBER,
                    gds_datatype = UNDERCUT_VERY_LOW_DOSE,
                    description = 'layer for very low dose ebeam undercut',
                    color = 'darkblue',
                    alpha = 0.8,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(undercut_very_low_layer)

UNDERCUT_LOW_DOSE = 400
UNDERCUT_LOW_LAYER_NUMBER = 17
undercut_low_layer = DesignLayer(
                    name = 'undercut_low',
                    gds_layer = UNDERCUT_LOW_LAYER_NUMBER,
                    gds_datatype = UNDERCUT_LOW_DOSE,
                    description = 'layer for low dose ebeam undercut',
                    color = 'darkblue',
                    alpha = 0.8,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'devices')
                    )

ebeam_layers.append(undercut_low_layer)

EBEAM_STRONG_DOSE = 1800
EBEAM_STRONG_LAYER_NUMBER = 60
ebeam_strong_layer = DesignLayer(
                    name = 'ebeam_strong',
                    gds_layer = EBEAM_STRONG_LAYER_NUMBER,
                    gds_datatype = EBEAM_STRONG_DOSE,
                    description = 'layer for high current ebeam lithography',
                    color = 'darkblue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam_large')
                    )

ebeam_layers.append(ebeam_strong_layer)

### Utility layers ###
######################

WRITEFIELD_EBEAM_LAYER_NUMBER = 70
writefield_ebeam_layer = DesignLayer(
                    name = 'writefield_ebeam',
                    gds_layer = WRITEFIELD_EBEAM_LAYER_NUMBER,
                    gds_datatype = 0,
                    description = 'layer for writefield',
                    color = 'brown',
                    alpha = 0.8,
                    layer_type = DesignLayerType.of('utility', 'ebeam', 'small_writefield')
                    )

ebeam_layers.append(writefield_ebeam_layer)

WRITEFIELD_EBEAM_STRONG_LAYER_NUMBER = 71
writefield_ebeam_strong_layer = DesignLayer(
                    name = 'writefield_ebeam_strong',
                    gds_layer = WRITEFIELD_EBEAM_STRONG_LAYER_NUMBER,
                    gds_datatype = 0,
                    description = 'layer for writefield at high current',
                    color = 'darkred',
                    alpha = 0.8,
                    layer_type = DesignLayerType.of('utility', 'ebeam', 'large_writefield')
                    )

ebeam_layers.append(writefield_ebeam_strong_layer)

WORKING_AREA_EBEAM_LAYER_NUMBER = 72
working_area_ebeam_layer = DesignLayer(
                    name = 'working_area_ebeam',
                    gds_layer = WORKING_AREA_EBEAM_LAYER_NUMBER,
                    gds_datatype = 0,
                    description = 'layer for ebeam working areas',
                    color = 'lightgreen',
                    alpha = 0.8,
                    layer_type = DesignLayerType.of('utility', 'ebeam', 'small_writefield')
                    )

ebeam_layers.append(working_area_ebeam_layer)

WORKING_AREA_EBEAM_STRONG_LAYER_NUMBER = 73
working_area_ebeam_strong_layer = DesignLayer(
                    name = 'working_area_ebeam_strong',
                    gds_layer = WORKING_AREA_EBEAM_STRONG_LAYER_NUMBER,
                    gds_datatype = 0,
                    description = 'layer for strong ebeam working areas',
                    color = 'green',
                    alpha = 0.8,
                    layer_type = DesignLayerType.of('utility', 'ebeam', 'large_writefield')
                    )

ebeam_layers.append(working_area_ebeam_strong_layer)

### Dose test (exposure) layers ###
###################################

EBEAM_TEST_LAYER_NUMBER = 25
UNDERCUT_TEST_LAYER_NUMBER = 35
EBEAM_LOG_TEST_LAYER_NUMBER = 40
UNDERCUT_LOG_TEST_LAYER_NUMBER = 50
EBEAM_FINE_TEST_LAYER_NUMBER = 45
UNDERCUT_FINE_TEST_LAYER_NUMBER = 55

### set range of doses for linear dose test, e.g. test_range = 0.3 to vary dose by ±30% around nominal dose
test_range = 0.3
num_ebeam_test_doses = 5
start_dose = EBEAM_JUNCTIONS_DOSE * (1 - test_range)
end_dose = EBEAM_JUNCTIONS_DOSE * (1 + test_range)
ebeam_test_doses = np.round(np.linspace(start_dose, end_dose, num_ebeam_test_doses, dtype=int), decimals=-1)

num_undercut_test_doses = 5
start_undercut_dose = JJ_UNDERCUT_DOSE * (1 - test_range)
end_undercut_dose = JJ_UNDERCUT_DOSE * (1 + test_range)
undercut_test_doses = np.round(np.linspace(start_undercut_dose, end_undercut_dose, num_undercut_test_doses, dtype=int), decimals=-1)


num_ebeam_log_test_doses = 5
start_log_dose = 1500
end_log_dose = 4500
ebeam_log_test_doses = np.geomspace(start_log_dose, end_log_dose, num_ebeam_log_test_doses, dtype=int)

num_undercut_log_test_doses = 5
start_log_undercut_dose = 150
end_log_undercut_dose = 1200
undercut_log_test_doses = np.geomspace(start_log_undercut_dose, end_log_undercut_dose, num_undercut_log_test_doses, dtype=int)


### set range of doses for fine linear dose test, e.g. test_range = 0.12 to vary dose by ±12% around nominal dose
fine_test_range = 0.12
num_ebeam_fine_test_doses = 5
start_fine_dose = EBEAM_JUNCTIONS_DOSE * (1 - fine_test_range)
end_fine_dose = EBEAM_JUNCTIONS_DOSE * (1 + fine_test_range)
ebeam_fine_test_doses = np.round(np.linspace(start_fine_dose, end_fine_dose, num_ebeam_fine_test_doses, dtype=int), decimals=-1)

num_undercut_fine_test_doses = 5
start_fine_undercut_dose = JJ_UNDERCUT_DOSE * (1 - fine_test_range)
end_fine_undercut_dose = JJ_UNDERCUT_DOSE * (1 + fine_test_range)
undercut_fine_test_doses = np.round(np.linspace(start_fine_undercut_dose, end_fine_undercut_dose, num_undercut_fine_test_doses, dtype=int), decimals=-1)

for i, dose in enumerate(ebeam_test_doses):
    ebeam_test_layer = DesignLayer(
                    name = f'ebeam_test_{i}',
                    gds_layer = EBEAM_TEST_LAYER_NUMBER+i,
                    gds_datatype = int(dose),
                    description = 'layer for ebeam lithography dose test',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'dose_test')
                    )

    ebeam_layers.append(ebeam_test_layer)

for i, undercut_dose in enumerate(undercut_test_doses):
    undercut_test_layer = DesignLayer(
                    name = f'undercut_test_{i}',
                    gds_layer = UNDERCUT_TEST_LAYER_NUMBER+i,
                    gds_datatype = int(undercut_dose),
                    description = 'layer for ebeam lithography undercut dose test',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'dose_test')
                    )

    ebeam_layers.append(undercut_test_layer)

for i, dose in enumerate(ebeam_log_test_doses):
    ebeam_log_test_layer = DesignLayer(
                    name = f'ebeam_log_test_{i}',
                    gds_layer = EBEAM_LOG_TEST_LAYER_NUMBER+i,
                    gds_datatype = int(dose),
                    description = 'layer for ebeam lithography logarithmic dose test',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'dose_test')
                    )

    ebeam_layers.append(ebeam_log_test_layer)

for i, undercut_dose in enumerate(undercut_log_test_doses):
    undercut_log_test_layer = DesignLayer(
                    name = f'undercut_log_test_{i}',
                    gds_layer = UNDERCUT_LOG_TEST_LAYER_NUMBER+i,
                    gds_datatype = int(undercut_dose),
                    description = 'layer for ebeam lithography logarithmic undercut dose test',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'dose_test')
                    )

    ebeam_layers.append(undercut_log_test_layer)

for i, dose in enumerate(ebeam_fine_test_doses):
    ebeam_fine_test_layer = DesignLayer(
                    name = f'ebeam_fine_test_{i}',
                    gds_layer = EBEAM_FINE_TEST_LAYER_NUMBER+i,
                    gds_datatype = int(dose),
                    description = 'layer for ebeam lithography fine dose test',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'dose_test')
                    )

    ebeam_layers.append(ebeam_fine_test_layer)

for i, undercut_dose in enumerate(undercut_fine_test_doses):
    undercut_fine_test_layer = DesignLayer(
                    name = f'undercut_fine_test_{i}',
                    gds_layer = UNDERCUT_FINE_TEST_LAYER_NUMBER+i,
                    gds_datatype = int(undercut_dose),
                    description = 'layer for ebeam lithography fine undercut dose test',
                    color = 'blue',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'ebeam', 'dose_test')
                    )

    ebeam_layers.append(undercut_fine_test_layer)

## define design type for this ebeam layer group
design_type = EbeamDesignGroup(nominal_dose_uC_cm2 = 300,
                               acceleration_voltage_kV= 30,
                               writefield_um = 96.0
                               )

## build ebeam layer group that can be used in the design
XL_30_SMALL_CURRENT_30kV_GROUP = DesignLayerGroup.build("XL_30_small_current_30kV_group",
                                                            ebeam_layers,
                                                            design_type,
                                                            DESIGN_LAYER_TYPE_TAXONOMY
                                                            )
